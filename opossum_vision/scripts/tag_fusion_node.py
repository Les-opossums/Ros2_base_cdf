#!/usr/bin/env python3
"""Noeud de fusion des tags ArUco en coordonnees MONDE.

Decouple de l'action_sequencer : il consomme les detections camera brutes
(`aruco_loc`) et la pose robot (`robot_data`), et publie :
  - `aruco_world`       : detections BRUTES recalees, a chaque trame camera
    (exploite 100% du debit camera) -> visualisation / debug.
  - `aruco_world_fused` : estimation STABLE et filtree par tag (fusion
    Hungarian + confirmation + dispersion), a 10 Hz -> utilisable en MATCH
    comme en debug.

Toute la logique de transformation/fusion est dans opossum_vision.tag_fusion
(bibliotheque pure, testee unitairement). Les constantes de reglage sont des
parametres ROS, modifiables a chaud (ros2 param set ...).
"""

import time
import math

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

from opossum_msgs.msg import VisionDataFrame, RobotData, GlobalView, Objects

from opossum_vision.tag_fusion import (
    PoseHistory, TagFuser, transform_to_world, aruco_color_name,
)


class TagFusionNode(Node):
    def __init__(self):
        super().__init__("tag_fusion_node")

        # --- Parametres reglables a chaud ---
        self.declare_parameter("camera_extra_latency_s", 0.0)
        self.declare_parameter("fuse_alpha", 0.35)
        self.declare_parameter("gate_m", 0.12)
        self.declare_parameter("match_m", 0.15)
        self.declare_parameter("min_hits", 3)
        self.declare_parameter("static_vlin", 0.03)
        self.declare_parameter("static_vt", 0.05)
        self.declare_parameter("absent_s", 2.0)
        self.declare_parameter("forget_s", 5.0)
        # Modele confiance vs mouvement
        self.declare_parameter("ref_vlin", 0.4)
        self.declare_parameter("ref_vt", 1.2)
        self.declare_parameter("rot_penalty", 2.5)
        self.declare_parameter("w_min", 0.05)
        self.declare_parameter("conf_beta", 0.2)
        self.declare_parameter("conf_tau_s", 1.5)
        self.declare_parameter("fused_rate_hz", 10.0)
        self.declare_parameter("near_reject_m2", 0.05)   # rejet chassis (dist^2)
        self.declare_parameter("near_reject_z", 0.17)    # ... si z au-dessus
        self.declare_parameter("far_reject_m", 1.0)      # rejet objets trop loin

        self.extra_latency = self.get_parameter("camera_extra_latency_s").value

        self.pose_history = PoseHistory(max_age_s=2.0)
        self.robot_speed = (0.0, 0.0)  # (vlin, vt)
        self.fuser = TagFuser()
        self._apply_fuser_params()

        # --- Publishers ---
        self.pub_world = self.create_publisher(GlobalView, "aruco_world", 20)
        self.pub_fused = self.create_publisher(GlobalView, "aruco_world_fused", 10)

        # --- Subscribers ---
        self.create_subscription(VisionDataFrame, "aruco_loc", self.aruco_callback, 20)
        self.create_subscription(RobotData, "robot_data", self.robot_data_callback, 20)

        rate = float(self.get_parameter("fused_rate_hz").value)
        self.create_timer(1.0 / max(rate, 1.0), self.publish_fused)

        self.add_on_set_parameters_callback(self._on_set_params)
        self.get_logger().info("tag_fusion_node demarre.")

    # ---------------- Parametres ----------------
    _TUNABLE = ("fuse_alpha", "gate_m", "match_m", "min_hits",
                "static_vlin", "static_vt", "absent_s", "forget_s",
                "ref_vlin", "ref_vt", "rot_penalty", "w_min",
                "conf_beta", "conf_tau_s")

    def _apply_fuser_params(self):
        self.fuser.set_config(
            alpha=self.get_parameter("fuse_alpha").value,
            gate_m=self.get_parameter("gate_m").value,
            match_m=self.get_parameter("match_m").value,
            min_hits=self.get_parameter("min_hits").value,
            static_vlin=self.get_parameter("static_vlin").value,
            static_vt=self.get_parameter("static_vt").value,
            absent_s=self.get_parameter("absent_s").value,
            forget_s=self.get_parameter("forget_s").value,
            ref_vlin=self.get_parameter("ref_vlin").value,
            ref_vt=self.get_parameter("ref_vt").value,
            rot_penalty=self.get_parameter("rot_penalty").value,
            w_min=self.get_parameter("w_min").value,
            conf_beta=self.get_parameter("conf_beta").value,
            conf_tau_s=self.get_parameter("conf_tau_s").value,
        )

    def _on_set_params(self, params):
        for p in params:
            if p.name == "camera_extra_latency_s":
                self.extra_latency = p.value
            elif p.name in self._TUNABLE:
                key = "alpha" if p.name == "fuse_alpha" else p.name
                self.fuser.set_config(**{key: p.value})
        return SetParametersResult(successful=True)

    # ---------------- Callbacks ----------------
    def robot_data_callback(self, msg: RobotData):
        self.pose_history.push(time.time(), msg.x, msg.y, msg.theta,
                               msg.vlin, msg.vdir, msg.vt)
        self.robot_speed = (msg.vlin, msg.vt)

    def aruco_callback(self, msg: VisionDataFrame):
        """Traite CHAQUE trame camera des reception."""
        if not getattr(msg, "object", None):
            return
        now = time.time()

        # Pose robot au moment de la capture (retard HEARTBEAT + latence reglable)
        cap = getattr(msg, "capture_time", 0.0)
        pose = None
        if cap > 0.0:
            pose, _ = self.pose_history.get_pose_at(cap - self.extra_latency)
        if pose is None:
            pose, _ = self.pose_history.get_pose_at(now)
        if pose is None:
            return

        near2 = self.get_parameter("near_reject_m2").value
        nearz = self.get_parameter("near_reject_z").value
        far = self.get_parameter("far_reject_m").value

        gv = GlobalView(); gv.robots = []; gv.objects = []
        dets = []
        for det in msg.object:
            r2 = det.x ** 2 + det.y ** 2
            if r2 < near2 and det.z > nearz:      # chassis
                continue
            if r2 > far ** 2:                      # trop loin
                continue
            wx, wy, wt = transform_to_world(pose, det.x, det.y, det.theta)
            color = aruco_color_name(det.id)

            o = Objects()
            o.id = int(det.id); o.type = color; o.state = f"cam{msg.id}"
            o.x = float(wx); o.y = float(wy); o.theta = float(wt)
            gv.objects.append(o)
            dets.append({"x": wx, "y": wy, "theta": wt, "color": color, "id": int(det.id)})

        self.pub_world.publish(gv)

        vlin, vt = self.robot_speed
        self.fuser.update(dets, now, vlin=vlin, vt=vt)

    def publish_fused(self):
        now = time.time()
        self.fuser.forget(now)
        gv = GlobalView(); gv.robots = []; gv.objects = []
        for e in self.fuser.snapshot(now, confirmed_only=True):
            o = Objects()
            o.id = int(e["key"])
            o.type = e["color"]
            o.state = (f"conf {int(round(e['confidence']*100))}%"
                       f" age {e['age_s']:.1f}s"
                       f" stat {int(round(e['std_static_m']*1000))}mm/{e['n_static']}"
                       f" mvt {int(round(e['std_moving_m']*1000))}mm/{e['n_moving']}")
            o.x = float(e["x"]); o.y = float(e["y"]); o.theta = float(e["theta"])
            o.confidence = float(e["confidence"])
            o.age_s = float(e["age_s"])
            o.present = bool(e["present"])
            gv.objects.append(o)
        self.pub_fused.publish(gv)


def main(args=None):
    rclpy.init(args=args)
    node = TagFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
