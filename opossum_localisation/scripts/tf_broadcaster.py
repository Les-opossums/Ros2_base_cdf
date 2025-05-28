#!/usr/bin/env python3
"""Broadcast all TF transforamtions."""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import functools
from opossum_msgs.msg import PositionMap, RobotData
from geometry_msgs.msg import Point
from std_srvs.srv import Empty
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from opossum_localisation.math_lidar import quaternion_from_euler


class TfBroadcaster(Node):
    """Broadcast all TF between frames."""

    def __init__(self):
        super().__init__("my_tf_broadcaster")
        self.declare_parameters(
            namespace="",
            parameters=[
                ("display_all", rclpy.Parameter.Type.BOOL),
                ("update_position_topic", rclpy.Parameter.Type.STRING),
                ("robot_data_topic", rclpy.Parameter.Type.STRING),
                ("visualization_topic", rclpy.Parameter.Type.STRING),
                ("default_color", rclpy.Parameter.Type.STRING),
                ("available_colors", rclpy.Parameter.Type.STRING_ARRAY),
                ("init_visu_srv", rclpy.Parameter.Type.STRING),
                ("boundaries", rclpy.Parameter.Type.DOUBLE_ARRAY),
                ("beacons", rclpy.Parameter.Type.DOUBLE_ARRAY),
                ("robot_names", rclpy.Parameter.Type.STRING_ARRAY),
                ("enable_wait_color", rclpy.Parameter.Type.BOOL),
                ("color_topic", rclpy.Parameter.Type.STRING),
            ],
        )
        self._init_main_parameters()

    def _init_main_parameters(self: Node) -> None:
        self.robot_names = (
            self.get_parameter("robot_names").get_parameter_value().string_array_value
        )
        self.available_colors = (
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
                '/' + self.robot_names[0] + '/' + self.color_topic,
                self._init_color_callback,
                10,
            )
        else:
            self.team_color = (
                self.get_parameter("default_color").get_parameter_value().string_value
            ).lower()
            if self.team_color not in self.available_colors:
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
        if color not in self.available_colors:
            raise ValueError("Invalid team color")
        self.team_color = color
        self._init_parameters()
        self._init_publishers()
        self._init_subscribers()

    def _init_parameters(self):
        """Initialize parameters."""
        self.br = TransformBroadcaster(self)
        self.display_all = (
            self.get_parameter("display_all").get_parameter_value().bool_value
        )
        self.boundaries = (
            self.get_parameter("boundaries").get_parameter_value().double_array_value
        )
        self.beacons = (
            self.get_parameter("beacons").get_parameter_value().double_array_value
        )
        self.robot_data_rcv = False
        if self.team_color not in self.available_colors:
            raise ValueError("Invalid team color")
        if self.team_color == "blue":
            for i in range(0, len(self.beacons), 2):
                self.beacons[i] = self.boundaries[1] - self.beacons[i]
        self.get_logger().info("TfBroadcaster node initialized.")
        

    def _init_subscribers(self):
        """Initialize subscribers."""
        update_position_topic = (
            self.get_parameter("update_position_topic")
            .get_parameter_value()
            .string_value
        )
        robot_data_topic = (
            self.get_parameter("robot_data_topic")
            .get_parameter_value()
            .string_value
        )
        for name in self.robot_names:
            self.sub_update_position = self.create_subscription(
                PositionMap,
                name + "/" + update_position_topic,
                functools.partial(self.broadcast_tf_from_update, name=name),
                10,
            )
            self.sub_robot_data = self.create_subscription(
                RobotData,
                name + "/" + robot_data_topic,
                functools.partial(self.broadcast_tf_from_robot_data, name=name),
                10,
            )

    def _init_publishers(self):
        """Initialize publishers."""
        visualization_topic = (
            self.get_parameter("visualization_topic").get_parameter_value().string_value
        )
        self.pub_visualization = self.create_publisher(Marker, visualization_topic, 10)

    def _init_services(self):
        """Initialize services."""
        init_visu_srv = (
            self.get_parameter("init_visu_srv").get_parameter_value().string_value
        )
        self.visualization_srv = self.create_service(
            Empty, init_visu_srv, self.update_display
        )

    def update_display(self, request, response):
        """Update the display of the board and beacons."""
        self._init_display()
        return response

    def _init_display(self):
        """Initialize the display of the board."""
        marker_board = Marker()
        marker_board.header.frame_id = "map"
        marker_board.header.stamp = self.get_clock().now().to_msg()
        marker_board.ns = "board"
        marker_board.id = 0
        marker_board.type = Marker.LINE_STRIP
        marker_board.action = Marker.ADD
        marker_board.scale.x = 0.01  # Line width

        # Color: red
        marker_board.color.g = 1.0
        marker_board.color.a = 1.0

        # Define the 8 points of a box (you can replace them)
        corners = [
            Point(x=self.boundaries[0], y=self.boundaries[2], z=0.0),
            Point(x=self.boundaries[1], y=self.boundaries[2], z=0.0),
            Point(x=self.boundaries[1], y=self.boundaries[3], z=0.0),
            Point(x=self.boundaries[0], y=self.boundaries[3], z=0.0),
            Point(x=self.boundaries[0], y=self.boundaries[2], z=0.0),
        ]
        marker_board.points = corners
        self.pub_visualization.publish(marker_board)
        for i in range(4):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "beacon"
            marker.id = i + 1
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD

            marker.scale.x = 0.01  # Line width
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.a = 1.0

            # Circle center and radius
            cx, cy, cz = self.beacons[2 * i], self.beacons[2 * i + 1], 0.0
            radius = 0.05
            num_points = 100  # More points = smoother circle

            for i in range(num_points + 1):  # Close the loop
                angle = 2 * math.pi * i / num_points
                x = cx + radius * math.cos(angle)
                y = cy + radius * math.sin(angle)
                marker.points.append(Point(x=x, y=y, z=cz))

            self.pub_visualization.publish(marker)

    def broadcast_tf_from_update(self, msg, name):
        """Link a frame to another."""
        if self.robot_data_rcv:
            return
        if self.display_all:
            self._init_display()
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = name + "/laser_frame"
        t.transform.translation.x = msg.robot.x
        t.transform.translation.y = msg.robot.y
        t.transform.translation.z = 0.0
        q = quaternion_from_euler(0.0, 0.0, msg.robot.z)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.br.sendTransform(t)

    def broadcast_tf_from_robot_data(self, msg, name):
        """Link a frame to another."""
        t = TransformStamped()
        if self.display_all:
            self._init_display()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = name + "/laser_frame"
        t.transform.translation.x = msg.x
        t.transform.translation.y = msg.y
        t.transform.translation.z = 0.0
        q = quaternion_from_euler(0.0, 0.0, msg.theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.br.sendTransform(t)


def main(args=None):
    """Spin ROS node."""
    rclpy.init(args=args)
    node = TfBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
