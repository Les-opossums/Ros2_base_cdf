#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterEvent

from std_msgs.msg import String

from opossum_action_sequencer.utils import *


class ActionManager(Node):
    def __init__(self):
        super().__init__("action_sequencer_node")
        self.get_logger().info("Action Manager Node started")
        self._init_parameters()
        self._init_publishers()
        self._init_subscribers()

    def _init_parameters(self) -> None:
        self.declare_parameters(
            namespace="",
            parameters=[
                ("pub_command", "command"),
                ("feedback_topic", "feedback_command"),
            ],
        )

    def _init_publishers(self):
        self.pub_command = self.create_publisher(String,
                                                 "command",
                                                 10
                                                 )
        self.pub_feedback = self.create_subscription(String,
                                                     "feedback_command",
                                                     self.feedback_callback,
                                                     10
                                                     )

    def _init_subscribers(self):
        self.subscription = self.create_subscription(
            ParameterEvent,
            '/parameter_events',
            self.parameter_event_callback,
            10)

    def parameter_event_callback(self, event):
        # Parcours des paramètres modifiés
        self.get_logger().info(f"Parameter event received: {event}")
        for changed in event.changed_parameters:
            if changed.name == 'script_number':
                # Affiche la nouvelle valeur du paramètre script_number
                self.get_logger().info(
                    f"Choix du script : {changed.value}"
                )
                run()


    def feedback_callback(self, msg):
        self.get_logger().info(f"Feedback received: {msg.data}")

        if msg.data.startswith("LEASH"):
            pass

    def move_to(self, pos: Position):
        self.get_logger().info(f"Moving to : {pos.x} {pos.y} {pos.t}")
        self.pub_command.publish(String(
            data=f"MOVE {pos.x} {pos.y} {pos.t}"
        ))


def main(args=None):
    rclpy.init(args=args)
    action_manager_node = ActionManager()
    rclpy.spin(action_manager_node)
    action_manager_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
