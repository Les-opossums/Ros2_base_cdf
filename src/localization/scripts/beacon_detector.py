#!/usr/bin/env python3
"""Detect beacons and return the best position match."""

# Libraries import
import rclpy
from rclpy.node import Node
import copy
import numpy as np

# Relative imports
from localization.math_lidar import get_angle_sign, dt, convert_world_to_robot
from localization.BeaconSorter import BeaconSorter
from rclpy.executors import ExternalShutdownException
from localization.PositionFinder import PositionFinder
from localization.publisher import publish_pose_from_lidar

# Msgs import
from cdf_msgs.msg import Obstacles, LidarLoc, MergedData
from std_msgs.msg import String


class BeaconDetectorNode(Node):
    """Detect the beacons and estimate the best matching position of the robot."""

    def __init__(self: Node) -> None:

        super().__init__("beacon_detector_node")
        self._init_main_parameters()
        self.get_logger().info("Beacon detector node initialized.")

    def _init_main_parameters(self: Node) -> None:
        """
        Initialise the main parameters of the node given by the launch file and its yaml config file.

        :param enable_wait_color: Enable the wait for the color message
        :type enable_wait_color: bool
        :param color_topic: The topic where the color message is published
        :type color_topic: str
        :param default_color: The default color of the team
        :type default_color: str
        :param team_color: The color of the team, chosen by default or by the message
        :type team_color: str
        :param available_colors: The available colors for the team
        :type available_colors: list
        """

        self.declare_parameters(
            namespace="",
            parameters=[
                ("enable_wait_color", rclpy.Parameter.Type.BOOL),
                ("color_topic", rclpy.Parameter.Type.STRING),
                ("available_colors", rclpy.Parameter.Type.STRING_ARRAY),
                ("default_color", rclpy.Parameter.Type.STRING),
                ("enable_robot_position_reception", rclpy.Parameter.Type.BOOL),
                ("enable_initial_position", rclpy.Parameter.Type.BOOL),
                ("distance_tolerance", rclpy.Parameter.Type.DOUBLE),
                ("angle_tolerance", rclpy.Parameter.Type.DOUBLE),
                ("init_position", rclpy.Parameter.Type.DOUBLE_ARRAY),
                ("boundaries", rclpy.Parameter.Type.DOUBLE_ARRAY),
                ("beacons", rclpy.Parameter.Type.DOUBLE_ARRAY),
                ("object_topic", rclpy.Parameter.Type.STRING),
                ("robot_position_topic", rclpy.Parameter.Type.STRING),
                ("position_topic", rclpy.Parameter.Type.STRING),
            ],
        )
        self._available_colors = (
            self.get_parameter("available_colors")
            .get_parameter_value()
            .string_array_value
        )
        if self.get_parameter("enable_wait_color").get_parameter_value().bool_value:
            self.color_topic = (
                self.get_parameter("color_topic").get_parameter_value().string_value
            )
            _ = self.create_subscription(
                String,
                self.color_topic,
                self._init_color_callback,
                10,
            )
        else:
            self.team_color = (
                self.get_parameter("default_color").get_parameter_value().string_value
            ).lower()
            if self.team_color not in self._available_colors:
                raise ValueError("Invalid team color")
            self._init_parameters()
            self._init_publishers()
            self._init_subscribers()

    def _init_color_callback(self: Node, msg: String) -> None:
        """
        Wait for the reception of the color message and set all the parameters once the message has been received.

        :param msg: The message containing the color of the team
        :type msg: String
        """
        color = msg.data.lower()
        if color not in self._available_colors:
            raise ValueError("Invalid team color")
        self.team_color = color
        self._init_parameters()
        self._init_publishers()
        self._init_subscribers()

    def _init_parameters(self: Node) -> None:
        """
        Initialise the parameters of the node given by the launch file and its yaml config file.

        :param enable_robot_position_reception: Enable the reception of the robot position once he's merged
        :type enable_robot_position_reception: bool
        :param enable_initial_position: Enable the initial position of the robot
        :type enable_initial_position: bool
        :param init_position: The initial position of the robot
        :type init_position: list
        :param dst_tol: The distance tolerance to consider a point is the same as the previous one
        :type dst_tol: float
        :param ang_tol: The angle tolerance to consider a point is the same as the previous one
        :type ang_tol: float
        :param boundaries: The boundaries of the playground [x_min, x_max, y_min, y_max]
        :type boundaries: list
        :param fixed_beacons: The beacons positions given by config as [x1, y1, x2, y2, x3, y3, x4, y4] and set as numpy lists
        :type fixed_beacons: list
        :param sign_vect_product: The sign of the vectorial product between the fixed beacons
        :type sign_vect_product: list
        :param position_finder: The position finder object
        :type position_finder: PositionFinder
        :param beacon_sorter: The beacon sorter object
        :type beacon_sorter: BeaconSorter
        """

        self.enable_robot_position_reception = (
            self.get_parameter("enable_robot_position_reception")
            .get_parameter_value()
            .bool_value
        )
        self.enable_initial_position = (
            self.get_parameter("enable_initial_position")
            .get_parameter_value()
            .bool_value
        )
        self.init_position = (
            (
                self.get_parameter("init_position")
                .get_parameter_value()
                .double_array_value
            )
            if self.enable_initial_position
            else None
        )
        self.dst_tol = (
            self.get_parameter("distance_tolerance").get_parameter_value().double_value
        )
        self.ang_tol = (
            self.get_parameter("angle_tolerance").get_parameter_value().double_value
        )
        self.boundaries = (
            self.get_parameter("boundaries").get_parameter_value().double_array_value
        )
        beacons = self.get_parameter("beacons").get_parameter_value().double_array_value
        if self.team_color == "blue":
            for i in range(1, len(beacons), 2):
                beacons[i] = self.boundaries[3] - beacons[i]
        self.fixed_beacons = [
            np.array([beacons[i], beacons[i + 1]]) for i in range(0, len(beacons), 2)
        ]
        self.sign_vect_product = [
            get_angle_sign(
                [
                    self.fixed_beacons[j]
                    for j in range(len(self.fixed_beacons))
                    if j != len(self.fixed_beacons) - 1 - i
                ]
            )
            for i in range(len(self.fixed_beacons))
        ]
        self.position_finder = PositionFinder(
            self.fixed_beacons,
            self.boundaries,
            self.dst_tol,
            self.init_position,
        )
        dst_beacons = {
            (chr(65 + i), chr(65 + j)): dt(self.fixed_beacons[i], self.fixed_beacons[j])
            for i in range(len(self.fixed_beacons))
            for j in range(i + 1, len(self.fixed_beacons))
        }
        self.beacon_sorter = BeaconSorter(
            dst_beacons,
            self.sign_vect_product,
            self.ang_tol,
            self.dst_tol,
        )

    def _init_subscribers(self: Node) -> None:
        """
        Initialise the subscribers of the node given by the launch file and its yaml config file.

        :param object_topic: The topic where the object message is published
        :type object_topic: str
        :param robot_position_topic: The topic where the robot merged position message is published
        :type robot_position_topic: str
        """

        self.object_topic = (
            self.get_parameter("object_topic").get_parameter_value().string_value
        )

        self.sub_object_topic = self.create_subscription(
            Obstacles,
            self.object_topic,
            self.object_callback,
            10,
        )
        if self.enable_robot_position_reception:
            self.robot_position_topic = (
                self.get_parameter("robot_position_topic")
                .get_parameter_value()
                .string_value
            )
            _ = self.create_subscription(
                MergedData,
                self.robot_position_topic,
                self.robot_position_callback,
                10,
            )

    def _init_publishers(self: Node) -> None:
        """
        Initialise the publishers of the node given by the launch file and its yaml config file.

        :param position_topic: The topic where the position message is published
        :type position_topic: str
        :param pub_location: The publisher to the position topic
        :type pub_location: Publisher
        :param test: If the node is in test mode. Used to display and check the results
        :type test: bool
        :param display_topic: The topic where the display message is published
        :type display_topic: str
        :param pub_display: The publisher to the display topic
        :type pub_display: Publisher
        """

        self.position_topic = (
            self.get_parameter("position_topic").get_parameter_value().string_value
        )
        self.pub_location = self.create_publisher(LidarLoc, self.position_topic, 10)

    def object_callback(self: Node, msg: Obstacles) -> None:
        """
        Detect the beacons and estimate the best matching position of the robot.

        :param msg: The message containing the detected objects
        :type msg: Obstacles
        """
        # begin = self.get_clock().now()
        if (
            not self.enable_robot_position_reception
            or self.position_finder.previous_robot is None
        ) and self.position_finder.current_robot is not None:
            self.position_finder.previous_robot = copy.deepcopy(
                self.position_finder.current_robot
            )

        previous_beacons = (
            self.position_finder.current_robot["beacons"]
            if self.position_finder.previous_robot is not None
            else None
        )
        new_objects_detected = [
            np.array([c.center.x, c.center.y])
            for c in msg.circles
            if c.center.x**2 + c.center.y**2 < 13.5
        ]
        (
            nb_potential_beacons,
            potential_beacons,
        ) = self.beacon_sorter._find_possible_beacons(
            previous_beacons,
            new_objects_detected,
        )
        if nb_potential_beacons > 0:
            position_found = self.position_finder.search_pos(
                nb_potential_beacons,
                potential_beacons,
                new_objects_detected,
            )
            if position_found is not None:
                self.pub_location.publish(publish_pose_from_lidar(position_found))

    def robot_position_callback(self: Node, msg: MergedData) -> None:
        """
        Update the previous position of the robot with the previous merged data.

        :param msg: The message containing the position of the robot
        :type msg: MergedData
        """
        # if self.position_finder.previous_robot is None:
        #     self.position_finder.previous_robot = {}
        # self.position_finder.previous_robot["position"] = np.array(
        #     [msg.robots[0].x, msg.robots[0].y, msg.robots[0].z]
        # )
        # self.position_finder.previous_robot["beacons"] = [
        #     convert_world_to_robot(
        #         fb, self.position_finder.previous_robot["position"]
        #     )
        #     for fb in self.fixed_beacons
        # ]
        self.position_finder.previous_robot = {
            "position": np.array([msg.robots[0].x, msg.robots[0].y, msg.robots[0].z]),
            "beacons": [
                convert_world_to_robot(
                    fb, np.array([msg.robots[0].x, msg.robots[0].y, msg.robots[0].z])
                )
                for fb in self.fixed_beacons
            ],
        }


# Main function, used as endpoint in the launch file
def main(args=None):
    """Run main loop."""
    rclpy.init(args=args)
    beacon_detector_node = BeaconDetectorNode()
    try:
        rclpy.spin(beacon_detector_node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        beacon_detector_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
