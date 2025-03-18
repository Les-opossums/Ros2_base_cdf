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
from cdf_msgs.msg import PositionMap
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
                ("robot_names", rclpy.Parameter.Type.STRING_ARRAY),
                ("real_position_topic", rclpy.Parameter.Type.STRING),
                ("trigger_position_srv", rclpy.Parameter.Type.STRING),
                ("update_period", rclpy.Parameter.Type.DOUBLE),
                ("update_position_topic", rclpy.Parameter.Type.STRING),
            ],
        )
        
        self.robot_names = (
            self.get_parameter("robot_names").get_parameter_value().string_array_value
        )
        self.update_period = (
            self.get_parameter("update_period").get_parameter_value().double_value
        )
        self.current_pos = {name: None for name in self.robot_names}

    def _init_publishers(self) -> None:
        self.update_position_topic = (
            self.get_parameter("update_position_topic").get_parameter_value().string_value
        )
        self.pub_update_position = {name: self.create_publisher(PositionMap, name + '/' +  self.update_position_topic, 10) for name in self.robot_names}

    def _init_subscribers(self):
        ##Inversion pb here, Weird 
        self.real_position_topic = (
            self.get_parameter("real_position_topic").get_parameter_value().string_value
        )

        #### WTF INIT LIKE THAT WORKS, BUT USING FOR LOOP DOES NOT !?????? 
        self.create_subscription(Point, '/main_robot/' + self.real_position_topic, lambda msg: self._update_pos(msg, "main_robot"), 10)
        self.create_subscription(Point, '/ennemi_robot/' + self.real_position_topic, lambda msg: self._update_pos(msg, "ennemi_robot"), 10)

        # for name in self.robot_names:
        #     self.get_logger().info(f"{name}: {'/' + name + '/' + self.real_position_topic}")
        #     sub_init = self.create_subscription(Point, '/' + name + '/' + self.real_position_topic, lambda msg: self._update_pos(msg, name), 10)
        #     sub_init
        self.create_timer(self.update_period, self._publish_position)

    def _init_clients(self):
        self.trigger_position_srv = (
            self.get_parameter("trigger_position_srv").get_parameter_value().string_value
        )
        for name in self.robot_names:
            cli_trigger_position = self.create_client(PosTrigger, name + '/' + self.trigger_position_srv)
            while not cli_trigger_position.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn(f'No position to update currently for {name}...')
            req = PosTrigger.Request()
            future = cli_trigger_position.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            self.current_pos[name] = future.result().pos

    def _update_pos(self, msg, name):
        self.get_logger().info(f"update of {name}")
        self.current_pos[name] = msg

    def _publish_position(self):
        for name in self.robot_names:
            self.get_logger().info(f"name: {name}")
            if self.current_pos[name] is not None:
                msg = PositionMap()
                msg.robot = self.current_pos[name]
                self.get_logger().info(f"name: {self.current_pos[name]}")
                for n in [na for na in self.robot_names if na != name]:
                    msg.ennemis.append(self.current_pos[n])
                self.pub_update_position[name].publish(msg)

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