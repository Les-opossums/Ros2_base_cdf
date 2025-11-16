#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Bool, Int32
from opossum_msgs.msg import RobotData, LidarLoc

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self._init_parameters()
        self._init_subscribers()
        self._init_publishers()

    def _init_parameters(self):
        """Initialize the parameters of the node."""
        self.declare_parameters(
            namespace="",
            parameters=[
                ("robot_data_topic", rclpy.Parameter.Type.STRING),
                ("robot_names", rclpy.Parameter.Type.STRING_ARRAY),
            ],
        )

        self.robot_names = (
            self.get_parameter("robot_names").get_parameter_value().string_array_value
        )

    def _init_subscribers(self):
        """Initialize the subscribers of the node."""
        robot_data_topic = (
            self.get_parameter("robot_data_topic")
            .get_parameter_value()
            .string_value
        )

        for name in self.robot_names:
            self.robot_data_sub = self.create_subscription(
                RobotData,
                name + "/" + robot_data_topic,
                self.robot_data_callback,
                10
            )

    def _init_publishers(self):
        pass

