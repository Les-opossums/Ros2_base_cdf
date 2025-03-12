#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from cdf_msgs.srv import Init
import tkinter as tk
from opossum_ihm.interface import GUI
from std_msgs.msg import Int32, Bool
import threading


class IhmNode(Node):
    def __init__(self):
        super().__init__('ihm_node')
        self.get_logger().info("Initializing IHM Node")

        # Initialisation des abonnements
        self._init_subscribers()
        self.current_score = 0

        # Initialisation de l'interface graphique
        self.gui = GUI()

        # Démarrage de la logique principale
        self.logic_thread = threading.Thread(target=self.main_logic, daemon=True)
        self.logic_thread.start()

    def main_logic(self):
        """Logique principale pour interagir avec l'utilisateur."""
        while True:
            self.gui.reload = False
            self.gui.run_color()
            if self.gui.reload:
                continue

            self.clr = self.gui.get_color()
            self.get_logger().info(f"Color selected: {self.clr}")

            self.gui.run_script()
            if self.gui.reload:
                continue

            self.scr = self.gui.get_script()
            self.get_logger().info(f"Script selected: {self.scr}")

            self.update_parameters()

            self.gui.run_validation()
            if self.gui.reload:
                continue
            else:
                self.update_parameters()
            break

        self.gui.initialized = True
        self.gui.run_score()
        self.gui.start_score()

    def update_parameters(self):
        """Met à jour les paramètres via un service ROS 2."""
        client = self.create_client(Init, 'set_parameters')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for 'set_parameters' service...")

        request = Init.Request()
        request.team_color = self.clr
        request.script_number = self.scr
        request.current_score = self.current_score

        future = client.call_async(request)
        future.add_done_callback(self.handle_service_response)

    def handle_service_response(self, future):
        """Gère la réponse du service asynchrone."""
        try:
            response = future.result()
            self.get_logger().info(f"Parameters updated successfully: {response}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def _init_subscribers(self):
        """Initialise les abonnements aux topics ROS 2."""
        self.get_logger().info("Setting up subscribers...")

        self.sub_score_topic = self.create_subscription(
            Int32,
            'score',
            self.score_callback,
            10
        )
        self.sub_au_topic = self.create_subscription(
            Bool,
            'au',
            self.au_callback,
            10
        )
        self.sub_enable_timer_topic = self.create_subscription(
            Bool,
            'enable_timer',
            self.enable_timer_callback,
            10
        )

    def score_callback(self, msg):
        """Callback pour le topic 'score'."""
        self.get_logger().info(f"Score received: {msg.data}")
        if self.gui.initialized:
            self.current_score += msg.data
            self.gui.score = self.current_score
            #self.gui.update_score(msg.data)
            self.get_logger().info(f"Score received: {msg.data}, current_score={self.current_score}")

    def au_callback(self, msg):
        """Callback pour le topic 'au'."""
        if self.gui.initialized:
            #self.gui.update_au(msg.data)
            self.get_logger().info(f"AU received: {msg.data}")

    def enable_timer_callback(self, msg):
        """Callback pour le topic 'enable_timer'."""
        if self.gui.initialized:
            self.gui.update_timer(msg.data)
            self.get_logger().info(f"Enable Timer received: {msg.data}")


def main(args=None):
    rclpy.init(args=args)
    node = IhmNode()

    try:
        rclpy.spin(node)  # Gestion des messages ROS 2
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
        node.get_logger().info("Shutting down IHM Node")


if __name__ == '__main__':
    main()
