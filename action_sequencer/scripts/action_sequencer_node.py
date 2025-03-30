#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from action_sequencer.action_manager import Version, Position, Speed
from action_sequencer.action_manager import PUMP_struct, LED_struct
from action_sequencer.action_manager import ODOM_struct, SERVO_struct


class ActionManager(Node):
    def __init__(self):
        super().__init__("action_manager_node")
        self._init_parameters()
        self._init_publishers()
        self.move_to(Position(1, 2, 3))

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
