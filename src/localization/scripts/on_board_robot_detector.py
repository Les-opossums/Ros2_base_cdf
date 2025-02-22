#!/usr/bin/env python3

# Import des librairies
import rclpy
from rclpy.node import Node
import copy
from localization.math_lidar import *
from localization.OnboardRobotFinder import OnboardRobotFinder
import time

# Import des messages
from visualization_msgs.msg import MarkerArray
from cdf_msgs.msg import Obstacles
from geometry_msgs.msg import Point
from cdf_msgs.msg import LidarLoc, MergedData
from std_msgs.msg import String
from localization.publisher import (
    publicate_donnees_lidar,
    publicate_donnees_zc,
    display_lidar_position,
    display_playground,
    diplay_fixed_beacons_positions,
    diplay_beacons_positions_found,
    display_other_robots_positions,
)
from localization.objet import RobotDatas


""" 
    Liste de potentielles améliorations:
        - Ameliorer le calcul de l'angle lorsque les méthodes dopti sont 
        supérieures à 1 en arccos et arcsin
        - Améliorer l'implémentation du code, l'utilisation des publishers et des 
        loginfos, et commenter les fonctions pour les plus courageux
"""


class OnboardRobotDetectorNode(Node):

    def __init__(self):
        super().__init__("onboard_robot_detector_node")
        self.test = True
        self._init_main_parameters()

    def _init_main_parameters(self):
        self.declare_parameters(
            namespace="",
            parameters=[
                ("beacons", [0.072, 3.094, 1.928, 3.094, 1., -0.094, -0.05, 1.725]),
                ("init_position", [-0.12, 1.725, 180.0]),
                ("boundaries", [0.0, 2.0, 0.0, 3.0]),
                ("object_topic", "zc_obstacle"),
                ("position_topic", "zc_position"),
                ("debug_topic", "debug"),
                ("display_topic", "zc_position_display"),
                ("color_topic", "init/team_color"),
                ("default_color", "yellow"),
                ("enable_wait_color", False)
            ],
        )
        self.enable_wait_color = (
            self.get_parameter("enable_wait_color").get_parameter_value().bool_value
        )
        if self.enable_wait_color:
            self.team_color = None
            self.color_topic = (
            self.get_parameter("color_topic").get_parameter_value().string_value
            )

            self.sub_color_topic = self.create_subscription(
                String,
                self.color_topic,
                self._init_color,
                10,
            )
        else:
            default_color = (
                self.get_parameter("default_color").get_parameter_value().string_value
            )
            self.team_color = default_color
            self._init_parameters()
            self._init_publishers()
            self._init_subscribers()

    def _init_color(self, msg) -> None:
        self.get_logger().info(f"Team color received: {msg.data}")
        if msg.data not in ["blue", "yellow"]:
            raise ValueError("Invalid team color")
        self.team_color = msg.data
        self._init_parameters()
        self._init_publishers()
        self._init_subscribers()

    def object_callback(self, msg: Obstacles) -> None:
        obstacles = [
            circle.center
            for circle in msg.circles
            if (circle.center.x**2 + circle.center.y**2) < 12.5
        ]
        robots = self.onboard_robot_finder.get_onboard_robot(obstacles)
        msg = publicate_donnees_zc(robots)
        self.pub_lidar_loc.publish(msg)
        if self.test:
            self.pub_debug.publish(String(data="Position trouvée"))
            self.pub_display.publish(display_playground(self.boundaries))
            self.pub_display.publish(display_other_robots_positions(robots))
            self.pub_display.publish(diplay_fixed_beacons_positions(self.fixed_beacons))

    def _init_parameters(self) -> None:
        self.boundaries = (
            self.get_parameter("boundaries").get_parameter_value().double_array_value
        )
        self.init_position = (
                self.get_parameter("init_position")
                .get_parameter_value()
                .double_array_value
        )
        beacons = (
            self.get_parameter("beacons").get_parameter_value().double_array_value
        )
        if self.team_color == "blue":
            for i in range(1, len(beacons), 2):
                beacons[i] = self.boundaries[3] - beacons[i]
            self.init_position[1] = 3 - self.init_position[1]
        self.fixed_beacons = [
            make_point([beacons[i], beacons[i + 1]]) for i in range(0, len(beacons), 2)
        ]
        self.onboard_robot_finder = OnboardRobotFinder(
            self.boundaries,
            self.init_position,
        )

    def _init_subscribers(self) -> None:
        self.object_topic = (
            self.get_parameter("object_topic").get_parameter_value().string_value
        )

        self.sub_object_topic = self.create_subscription(
            Obstacles,
            self.object_topic,
            self.object_callback,
            10,
        )

    def _init_publishers(self) -> None:
        self.position_topic = (
            self.get_parameter("position_topic").get_parameter_value().string_value
        )
        self.pub_lidar_loc = self.create_publisher(LidarLoc, self.position_topic, 10)
        if self.test:
            self.debug_topic = (
                self.get_parameter("debug_topic").get_parameter_value().string_value
            )
            self.display_topic = (
                self.get_parameter("display_topic").get_parameter_value().string_value
            )
            self.pub_debug = self.create_publisher(String, self.debug_topic, 10)
            self.pub_display = self.create_publisher(
                MarkerArray, self.display_topic, 10
            )
            self.pub_debug.publish(String(data="Initialisation"))


def main():
    rclpy.init()
    node = OnboardRobotDetectorNode()
    rclpy.spin(node)
    rclpy.shutdown()
