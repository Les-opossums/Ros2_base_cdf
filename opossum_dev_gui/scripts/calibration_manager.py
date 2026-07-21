#!/usr/bin/env python3
"""Gestionnaire de calibration camera, pilotable depuis l'IHM web.

Recoit des commandes sur `calibration/command` (std_msgs/String) et publie son
etat sur `calibration/status` (std_msgs/String, JSON). Il sait :
  - enregistrer un rosbag des topics utiles (aruco_loc + robot_data + ...),
  - jouer une petite sequence de mouvements (translations + rotations, a l'arret
    et en mouvement) devant un tag fixe, pour exciter l'erreur de recalage
    dependante du retard camera,
  - faire les deux d'un coup (RUN : bag start -> mouvements -> bag stop).

Le bag produit est ensuite analyse hors-ligne par le script d'auto-calibration
(Opossum_vision_calibration) qui trouve la latence camera optimale.

Commandes acceptees (champ data) :
  RUN         : demarre un bag + joue la sequence + arrete le bag
  BAG_START   : demarre juste l'enregistrement
  BAG_STOP    : arrete l'enregistrement
  MOVE_TEST   : joue juste la sequence de mouvements (sans bag)
  ABORT       : stoppe tout (sequence + bag)
"""

import os
import json
import time
import signal
import threading
import subprocess
import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from opossum_msgs.msg import LidarLoc


class CalibrationManager(Node):
    def __init__(self):
        super().__init__("calibration_manager")

        self.declare_parameter("bag_dir", "/home/opossum/robot_ws/calib_bags")
        self.declare_parameter("command_topic", "command")
        # Amplitudes de la sequence (m et rad), modestes et reglables.
        self.declare_parameter("amp_lat", 0.20)
        self.declare_parameter("amp_fwd", 0.15)
        self.declare_parameter("amp_rot", 0.5)
        self.declare_parameter("settle_s", 1.5)

        self.bag_dir = self.get_parameter("bag_dir").value
        cmd_topic = self.get_parameter("command_topic").value

        self.ns = self.get_namespace().rstrip("/")  # ex: /main_robot
        self.robot_pose = None                       # (x, y, theta)
        self.bag_proc = None
        self.bag_path = None
        self._abort = threading.Event()
        self._seq_thread = None
        self.state = "idle"
        self.step = ""

        self.pub_cmd = self.create_publisher(String, cmd_topic, 10)
        self.pub_status = self.create_publisher(String, "calibration/status", 10)
        self.create_subscription(LidarLoc, "position_out", self._pose_cb, 10)
        self.create_subscription(String, "calibration/command", self._command_cb, 10)
        self.create_timer(0.5, self._publish_status)

        self.get_logger().info("calibration_manager pret.")

    # ---------------- Etat ----------------
    def _publish_status(self, extra=""):
        payload = {
            "state": self.state, "step": self.step,
            "bag": self.bag_path or "", "recording": self.bag_proc is not None,
            "pose_ok": self.robot_pose is not None, "msg": extra,
        }
        self.pub_status.publish(String(data=json.dumps(payload)))

    def _set_state(self, state, step="", msg=""):
        self.state = state
        self.step = step
        self.get_logger().info(f"[calib] {state} {step} {msg}")
        self._publish_status(msg)

    # ---------------- Callbacks ----------------
    def _pose_cb(self, msg: LidarLoc):
        p = msg.robot_position
        self.robot_pose = (p.x, p.y, p.z)  # z = theta

    def _command_cb(self, msg: String):
        cmd = msg.data.strip().upper()
        if cmd == "RUN":
            self._start_sequence(record=True)
        elif cmd == "MOVE_TEST":
            self._start_sequence(record=False)
        elif cmd == "BAG_START":
            self._bag_start()
        elif cmd == "BAG_STOP":
            self._bag_stop()
        elif cmd == "ABORT":
            self._abort_all()
        else:
            self._set_state(self.state, self.step, f"commande inconnue: {cmd}")

    # ---------------- Rosbag ----------------
    def _bag_topics(self):
        n = self.ns if self.ns else ""
        return [f"{n}/aruco_loc", f"{n}/robot_data", f"{n}/position_out",
                f"{n}/aruco_world", f"{n}/aruco_world_fused", f"{n}/command"]

    def _bag_start(self):
        if self.bag_proc is not None:
            self._set_state(self.state, self.step, "bag deja en cours")
            return False
        os.makedirs(self.bag_dir, exist_ok=True)
        stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.bag_path = os.path.join(self.bag_dir, f"calib_{stamp}")
        cmd = ["ros2", "bag", "record", "-o", self.bag_path] + self._bag_topics()
        try:
            self.bag_proc = subprocess.Popen(cmd, start_new_session=True)
        except Exception as e:
            self.bag_proc = None
            self._set_state("error", "bag", f"echec record: {e}")
            return False
        self._set_state("recording", "bag", f"-> {self.bag_path}")
        return True

    def _bag_stop(self):
        if self.bag_proc is None:
            return
        try:
            os.killpg(os.getpgid(self.bag_proc.pid), signal.SIGINT)
            self.bag_proc.wait(timeout=8)
        except Exception:
            try:
                os.killpg(os.getpgid(self.bag_proc.pid), signal.SIGKILL)
            except Exception:
                pass
        self.bag_proc = None
        self._set_state("idle", "", f"bag enregistre: {self.bag_path}")

    # ---------------- Sequence de mouvements ----------------
    def _send(self, raw):
        self.pub_cmd.publish(String(data=raw))

    def _move_rel(self, dx, dy, dth, vmax):
        if self.robot_pose is None:
            return
        x0, y0, t0 = self.robot_pose
        self._send(f"VMAX {vmax}")
        self._send(f"MOVE {x0 + dx:.3f} {y0 + dy:.3f} {t0 + dth:.3f}")

    def _start_sequence(self, record):
        if self._seq_thread and self._seq_thread.is_alive():
            self._set_state(self.state, self.step, "sequence deja en cours")
            return
        if self.robot_pose is None:
            self._set_state("error", "", "position robot inconnue (position_out ?)")
            return
        self._abort.clear()
        self._seq_thread = threading.Thread(
            target=self._run_sequence, args=(record,), daemon=True)
        self._seq_thread.start()

    def _sleep(self, s):
        """Sleep interruptible par ABORT. Retourne False si abort."""
        end = time.time() + s
        while time.time() < end:
            if self._abort.is_set():
                return False
            time.sleep(0.05)
        return True

    def _run_sequence(self, record):
        lat = self.get_parameter("amp_lat").value
        fwd = self.get_parameter("amp_fwd").value
        rot = self.get_parameter("amp_rot").value
        settle = self.get_parameter("settle_s").value

        # (label, dx, dy, dtheta, vmax, wait)  -- offsets relatifs au depart
        seq = [
            ("repos initial", 0, 0, 0, 0.4, 2.0),
            ("lateral +", 0, lat, 0, 0.4, settle),
            ("retour", 0, 0, 0, 0.4, settle),
            ("lateral -", 0, -lat, 0, 0.4, settle),
            ("retour", 0, 0, 0, 0.4, settle),
            ("rotation +", 0, 0, rot, 0.4, settle),
            ("rotation -", 0, 0, -rot, 0.4, settle),
            ("retour", 0, 0, 0, 0.4, settle),
            ("rapide +", fwd, 0, 0, 0.8, 1.2),
            ("rapide -", -fwd, 0, 0, 0.8, 1.2),
            ("retour", 0, 0, 0, 0.4, settle),
            ("repos final", 0, 0, 0, 0.4, 2.0),
        ]

        if record and not self._bag_start():
            return
        if record:
            self._sleep(1.0)  # laisse le bag demarrer

        for i, (label, dx, dy, dth, vmax, wait) in enumerate(seq):
            if self._abort.is_set():
                break
            self._set_state("moving", f"{i+1}/{len(seq)} {label}")
            self._move_rel(dx, dy, dth, vmax)
            if not self._sleep(wait):
                break

        aborted = self._abort.is_set()
        if record:
            self._bag_stop()
        if aborted:
            self._set_state("idle", "", "sequence interrompue (ABORT)")
        else:
            self._set_state("done", "", "sequence terminee"
                            + (f" -> bag {self.bag_path}" if record else ""))

    def _abort_all(self):
        self._abort.set()
        self._send("STOP")
        self._bag_stop()
        self._set_state("idle", "", "ABORT")

    def destroy_node(self):
        self._abort.set()
        self._bag_stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CalibrationManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
