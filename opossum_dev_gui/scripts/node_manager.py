#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Superviseur de noeuds (node_manager).

Permet d'ALLUMER / COUPER les noeuds du robot a la demande depuis l'IHM web,
et publie l'etat de presence de chaque noeud (vu dans le graphe ROS 2) pour la
page « Systeme ».

Contrat avec l'IHM web (topics relatifs au namespace, ex: /main_robot) :

  * Commande recue : ``node_manager/command``  (std_msgs/String, JSON)
        {"action": "start",   "key": "vision_one"}
        {"action": "stop",    "key": "vision_one"}
        {"action": "restart", "key": "vision_one"}
        {"action": "start_all"} / {"action": "stop_all"}

  * Statut publie : ``node_manager/status``  (std_msgs/String, JSON) a 1 Hz
        {
          "stamp": <epoch>,
          "nodes": [
            {"key":"vision_one","label":"Vision cam 1","group":"vision",
             "present": true,       # noeud vu dans le graphe ROS
             "managed": true,       # sous-process gere par ce superviseur
             "running": true,       # process encore vivant
             "pid": 12345},
            ...
          ]
        }

Le champ ``present`` est fiable meme si le noeud a ete lance par un bringup
externe (detection via le graphe ROS). ``managed``/``running`` refletent les
process que CE superviseur a lances (donc qu'il peut arreter).

La liste des noeuds et leur commande de lancement sont decrites dans
``config/node_manager.yaml``.
"""

import json
import os
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ManagedNode:
    def __init__(self, spec, default_ns):
        self.key = spec["key"]
        self.label = spec.get("label", self.key)
        self.group = spec.get("group", "divers")
        self.package = spec["package"]
        self.executable = spec["executable"]
        self.node_name = spec.get("node_name", self.key)
        self.namespace = spec.get("namespace", default_ns)
        self.params = spec.get("params", "")
        self.autostart = bool(spec.get("autostart", False))
        self.proc = None  # subprocess.Popen si gere par nous

    # --- Cycle de vie ---
    def build_cmd(self):
        cmd = ["ros2", "run", self.package, self.executable, "--ros-args"]
        if self.namespace:
            cmd += ["-r", f"__ns:=/{self.namespace.strip('/')}"]
        if self.node_name:
            cmd += ["-r", f"__node:={self.node_name}"]
        if self.params:
            cmd += ["--params-file", self.params]
        return cmd

    def start(self, logger):
        if self.is_running():
            return False
        try:
            self.proc = subprocess.Popen(self.build_cmd(), start_new_session=True)
            logger.info(f"[node_manager] start {self.key} (pid={self.proc.pid})")
            return True
        except Exception as e:  # noqa: BLE001
            self.proc = None
            logger.error(f"[node_manager] echec start {self.key} : {e}")
            return False

    def stop(self, logger):
        if self.proc is None:
            return False
        try:
            os.killpg(os.getpgid(self.proc.pid), signal.SIGINT)
            try:
                self.proc.wait(timeout=5)
            except Exception:
                os.killpg(os.getpgid(self.proc.pid), signal.SIGKILL)
            logger.info(f"[node_manager] stop {self.key}")
        except Exception as e:  # noqa: BLE001
            logger.warn(f"[node_manager] echec stop {self.key} : {e}")
        finally:
            self.proc = None
        return True

    def is_running(self):
        return self.proc is not None and self.proc.poll() is None


class NodeManager(Node):
    def __init__(self):
        super().__init__("node_manager")
        self.declare_parameter("namespace_default", "main_robot")
        self.default_ns = self.get_parameter("namespace_default").get_parameter_value().string_value or "main_robot"

        self.nodes = self._load_registry()

        self.pub_status = self.create_publisher(String, "node_manager/status", 10)
        self.create_subscription(String, "node_manager/command", self.on_command, 10)
        self.create_timer(1.0, self.publish_status)

        # Auto-demarrage des noeuds flagges (defaut: aucun)
        for n in self.nodes.values():
            if n.autostart:
                n.start(self.get_logger())

        self.get_logger().info(f"[node_manager] {len(self.nodes)} noeud(s) au registre.")

    def _load_registry(self):
        """Charge la liste des noeuds depuis le parametre 'nodes' (YAML)."""
        nodes = {}
        specs = []
        try:
            self.declare_parameter("nodes", rclpy.Parameter.Type.STRING_ARRAY)
        except Exception:
            pass
        # Les dictionnaires imbriques YAML ne passent pas en parametres ROS
        # simples : on lit le YAML directement pour rester robuste.
        import glob
        from ament_index_python.packages import get_package_share_directory
        try:
            share = get_package_share_directory("opossum_dev_gui")
            path = os.path.join(share, "config", "node_manager.yaml")
            specs = self._parse_yaml_registry(path)
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"[node_manager] lecture registre impossible : {e}")
            specs = []
        for spec in specs:
            try:
                mn = ManagedNode(spec, self.default_ns)
                nodes[mn.key] = mn
            except Exception as e:  # noqa: BLE001
                self.get_logger().warn(f"[node_manager] entree invalide {spec} : {e}")
        return nodes

    @staticmethod
    def _parse_yaml_registry(path):
        import yaml
        with open(path) as f:
            data = yaml.safe_load(f)
        # Structure : <ns>/node_manager/ros__parameters/nodes
        for root in data.values():
            nm = root.get("node_manager", {}) if isinstance(root, dict) else {}
            params = nm.get("ros__parameters", {})
            if "nodes" in params:
                return params["nodes"]
        return []

    # --- Presence dans le graphe ROS ---
    def _present_node_names(self):
        try:
            return {name for name, _ns in self.get_node_names_and_namespaces()}
        except Exception:
            return set()

    # --- Commandes ---
    def on_command(self, msg):
        try:
            cmd = json.loads(msg.data)
        except Exception:
            self.get_logger().warn(f"[node_manager] commande JSON invalide : {msg.data!r}")
            return
        action = cmd.get("action", "")
        key = cmd.get("key")
        log = self.get_logger()
        if action == "start_all":
            for n in self.nodes.values():
                n.start(log)
        elif action == "stop_all":
            for n in self.nodes.values():
                n.stop(log)
        elif key in self.nodes:
            n = self.nodes[key]
            if action == "start":
                n.start(log)
            elif action == "stop":
                n.stop(log)
            elif action == "restart":
                n.stop(log)
                time.sleep(0.5)
                n.start(log)
            else:
                log.warn(f"[node_manager] action inconnue : {action}")
        else:
            log.warn(f"[node_manager] noeud inconnu : {key}")
        self.publish_status()

    # --- Statut ---
    def publish_status(self):
        present = self._present_node_names()
        out = {"stamp": time.time(), "nodes": []}
        for n in self.nodes.values():
            out["nodes"].append({
                "key": n.key,
                "label": n.label,
                "group": n.group,
                "node_name": n.node_name,
                "present": n.node_name in present,
                "managed": n.proc is not None,
                "running": n.is_running(),
                "pid": (n.proc.pid if n.is_running() else None),
            })
        msg = String()
        msg.data = json.dumps(out)
        self.pub_status.publish(msg)

    def destroy_node(self):
        # Coupe proprement les process qu'on a lances.
        for n in self.nodes.values():
            n.stop(self.get_logger())
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = NodeManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
