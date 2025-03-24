#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class ActionManager(Node):
    def __init__(self):
        super().__init__("action_manager_node")

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

    def feedback_callback(self, msg):
        self.get_logger().info(f"Feedback received: {msg.data}")


def main(args=None):
    rclpy.init(args=args)
    action_manager_node = ActionManager()
    rclpy.spin(action_manager_node)
    action_manager_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
