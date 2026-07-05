#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import os

# --- FIX POUR UBUNTU / WAYLAND / SNAP ---
os.environ["QT_QPA_PLATFORM"] = "xcb"
os.environ["QT_QPA_PLATFORMTHEME"] = ""
if "GTK_PATH" in os.environ:
    del os.environ["GTK_PATH"]
# ----------------------------------------

import rclpy
from rclpy.node import Node
from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import QThread, pyqtSignal

from opossum_ihm.interface import MainWindow
from opossum_msgs.msg import LidarLoc, RobotData
from opossum_msgs.srv import Init
from std_srvs.srv import Trigger 
from std_msgs.msg import Int32, Bool, String

class RosNode(Node):
    def __init__(self):
        super().__init__("ihm_node")
        
        # --- Variables partagées (remplacent les signaux haute fréquence) ---
        self.latest_lidar = (0.0, 0.0, 0.0)
        self.latest_zynq = (0.0, 0.0, 0.0)
        self.latest_cams = {1: (0.0, 0.0, 0.0), 2: (0.0, 0.0, 0.0), 3: (0.0, 0.0, 0.0)}

        # --- Clients / Publishers ---
        self.param_client = self.create_client(Init, "set_parameters")
        self.reset_match_client = self.create_client(Trigger, "reset_match")
        self.pub_color = self.create_publisher(String, "init_team_color", 10)

    def publish_color(self, color):
        msg = String()
        msg.data = color
        self.pub_color.publish(msg)

    def send_parameters(self, color, script):
        if not self.param_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Service set_parameters non disponible")
            return
        req = Init.Request()
        req.team_color = color
        req.script_number = script
        self.param_client.call_async(req)


class RosThread(QThread):
    # Uniquement les signaux événementiels ou basse fréquence
    sig_score = pyqtSignal(int)
    sig_au = pyqtSignal(bool)
    sig_comm_state = pyqtSignal(bool)
    sig_feedback_command = pyqtSignal(str) 

    def __init__(self):
        super().__init__()
        # 1. CRÉATION DE L'ENVIRONNEMENT ROS DÈS L'INITIALISATION
        rclpy.init()
        self.node = RosNode()
        
        # 2. Abonnements lents -> Signaux PyQt
        self.node.create_subscription(Int32, "score", lambda m: self.sig_score.emit(m.data), 10)
        self.node.create_subscription(Bool, "au", lambda m: self.sig_au.emit(m.data), 10)
        self.node.create_subscription(Bool, "comm_state", lambda m: self.sig_comm_state.emit(m.data), 10)
        self.node.create_subscription(String, "feedback_command", lambda m: self.sig_feedback_command.emit(m.data), 10)
        
        # 3. Abonnements rapides -> Mise à jour directe des variables (thread-safe)
        self.node.create_subscription(String, "command", self.cb_cam, 10)
        self.node.create_subscription(LidarLoc, "position_out", self.cb_lidar, 1)
        self.node.create_subscription(RobotData, "robot_data", self.cb_zynq, 1)

    def cb_lidar(self, msg):
        self.node.latest_lidar = (msg.robot_position.x, msg.robot_position.y, msg.robot_position.z)

    def cb_zynq(self, msg):
        self.node.latest_zynq = (msg.x, msg.y, msg.theta)

    def cb_cam(self, msg):
        d = msg.data.strip().split()
        if d and d[0].startswith("SETCAMERA"):
            try:
                cam_id = int(d[0].replace("SETCAMERA", ""))
                self.node.latest_cams[cam_id] = (float(d[1]), float(d[2]), float(d[3]))
            except:
                pass

    def run(self):
        # 4. LE THREAD NE FAIT PLUS QUE TOURNER LE NOEUD (SingleThreadedExecutor implicite)
        rclpy.spin(self.node)

    def stop(self):
        if hasattr(self, 'node'):
            self.node.destroy_node()
        rclpy.shutdown()
        self.quit()
        self.wait()


def main():
    app = QApplication(sys.argv)

    # --- STYLE GLOBAL ---
    app.setStyleSheet("""
        QMessageBox { background-color: #F0F0F0; }
        QMessageBox QLabel {
            font-size: 24px; font-weight: bold; color: black;
            min-width: 420px; margin: 20px;
        }
        QMessageBox QPushButton {
            font-size: 22px; font-weight: bold;
            min-width: 180px; min-height: 100px;
            border-radius: 12px; border: 2px solid #333333;
            background-color: #DDDDDD; margin: 10px;
        }
        QMessageBox QPushButton:pressed { background-color: #999999; }
        QPushButton { font-size: 18px; font-weight: bold; }
    """)

    # Initialisation du thread ROS (le noeud est créé instantanément)
    ros_thread = RosThread()
    
    # Création de l'IHM
    window = MainWindow()

    # CRUCIAL : Le noeud existant déjà, l'injection fonctionne parfaitement !
    window.page_match.ros_node_ref = ros_thread.node

    # Lancement du thread ROS (qui va appeler 'run' en tâche de fond)
    ros_thread.start()

    # Branchements ROS -> IHM (Signaux restants)
    ros_thread.sig_score.connect(window.page_match.update_score)
    ros_thread.sig_au.connect(window.page_match.set_au_state)
    ros_thread.sig_comm_state.connect(window.page_match.set_comm_state)
    ros_thread.sig_feedback_command.connect(window.page_match.set_match_state)

    # Branchements IHM -> ROS
    window.page_match.request_restart_match.connect(
        lambda: ros_thread.node.reset_match_client.call_async(Trigger.Request()) if hasattr(ros_thread, 'node') else None
    )
    
    window.page_config.request_param_update.connect(
        lambda c, s: [
            ros_thread.node.publish_color(c), 
            ros_thread.node.send_parameters(c, s), 
            window.go_to_match_page(c)
        ]
    )

    window.show()

    app.aboutToQuit.connect(ros_thread.stop)
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()