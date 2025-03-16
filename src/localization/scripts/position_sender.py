#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
import time
import numpy as np
import matplotlib.pyplot as plt
from rclpy.executors import MultiThreadedExecutor
from rclpy.executors import ExternalShutdownException
from rclpy.callback_groups import ReentrantCallbackGroup

# Import des messages
from cdf_msgs.action import MoveTo
from cdf_msgs.srv import PosTrigger
from geometry_msgs.msg import Point
import threading


class PositionSender(Node):
    def __init__(self):
        super().__init__("position_sender_node")
        self._init_parameters()
        self._init_publishers()
        self._init_subscribers()
        self._init_clients()
        self.get_logger().info(f"Position sender node initialized.")

    def _init_parameters(self) -> None:
        self.declare_parameters(
            namespace="",
            parameters=[
                ("real_position_topic", rclpy.Parameter.Type.STRING),
                ("trigger_position_srv", rclpy.Parameter.Type.STRING),
                ("update_period", rclpy.Parameter.Type.DOUBLE),
                ("update_position_topic", rclpy.Parameter.Type.STRING),
            ],
        )
        self.update_period = (
            self.get_parameter("update_period").get_parameter_value().double_value
        )
        self.current_pos = None

    def _init_publishers(self) -> None:
        self.update_position_topic = (
            self.get_parameter("update_position_topic").get_parameter_value().string_value
        )
        self.pub_update_position = self.create_publisher(Point, self.update_position_topic, 10)

    def _init_subscribers(self):
        self.real_position_topic = (
            self.get_parameter("real_position_topic").get_parameter_value().string_value
        )
        self.create_subscription(Point, self.real_position_topic, self._update_pos, 10)
        self.create_timer(self.update_period, self._publish_position)

    def _init_clients(self):
        self.trigger_position_srv = (
            self.get_parameter("trigger_position_srv").get_parameter_value().string_value
        )
        self.cli_trigger_position = self.create_client(PosTrigger, self.trigger_position_srv)
        while not self.cli_trigger_position.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('No position to update currently...')
        self.req = PosTrigger.Request()
        self.future = self.cli_trigger_position.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        self.current_pos = self.future.result().pos

    def _update_pos(self, msg):
        self.current_pos = msg

    def _publish_position(self):
        if self.current_pos is not None:
            self.pub_update_position.publish(self.current_pos)

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