#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

# Import des messages
from geometry_msgs.msg import Point

class PositionSender(Node):
    def __init__(self):
        super().__init__("position_sender_node")
        self._init_parameters()
        self._init_publishers()
        self.init_subscribers()

    def _init_parameters(self) -> None:
        self.declare_parameters(
            namespace="",
            parameters=[
                ("real_position_topic", rclpy.Parameter.Type.STRING),
                ("update_position_topic", rclpy.Parameter.Type.STRING),
                ("update_period", rclpy.Parameter.Type.DOUBLE),
            ],
        )
        self.update_period = (
            self.get_parameter("update_period").get_parameter_value().double_value
        )
        self.current_state = None

    def _init_publishers(self) -> None:
        self.real_position_topic = (
            self.get_parameter("real_position_topic").get_parameter_value().string_value
        )
        self.pub_real_position = self.create_publisher(Point, self.real_position_topic, 10)

    def init_subscribers(self):
        self.update_position_topic = self.get_parameter("update_position_topic").get_parameter_value().string_value
        self.create_timer(self.update_period, self._publish_position)
        self.create_subscription(Point, self.update_position_topic, self._update_position, 10)

    def _update_position(self, msg):
        self.current_state = msg

    def _publish_position(self):
        if self.current_state is None:
            return 
        self.pub_real_position.publish(self.current_state)

def main(args=None):
    rclpy.init(args=args)
    position_sender_node = PositionSender()
    try:
        rclpy.spin(position_sender_node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        position_sender_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()