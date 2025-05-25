#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Receive all the world information and send it to every captors (lidars at now)."""

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

# Import des messages
from opossum_msgs.srv import StringReq
from geometry_msgs.msg import Point
from opossum_msgs.msg import PositionMap
import functools


class PositionSender(Node):
    """Receive all the world information and send it to every captors (lidars at now)."""

    def __init__(self):
        super().__init__("position_sender_node")
        self._init_parameters()
        self._init_publishers()
        self._init_subscribers()
        self._init_clients()
        self.get_logger().info("Position sender node initialized.")

    def _init_parameters(self) -> None:
        """Initialize parameters of the node."""
        self.declare_parameters(
            namespace="",
            parameters=[
                ("robot_names", rclpy.Parameter.Type.STRING_ARRAY),
                ("real_position_topic", rclpy.Parameter.Type.STRING),
                ("short_motor_srv", rclpy.Parameter.Type.STRING),
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
        """Initialize publishers of the node."""
        self.update_position_topic = (
            self.get_parameter("update_position_topic")
            .get_parameter_value()
            .string_value
        )
        self.pub_update_position = {
            name: self.create_publisher(
                PositionMap, name + "/" + self.update_position_topic, 10
            )
            for name in self.robot_names
        }

    def _init_subscribers(self):
        """Initialize subscribers of the node."""
        self.real_position_topic = (
            self.get_parameter("real_position_topic").get_parameter_value().string_value
        )

        for name in self.robot_names:
            sub_init = self.create_subscription(
                Point,
                "/" + name + "/" + self.real_position_topic,
                functools.partial(self._update_pos, name=name),
                10,
            )
            sub_init
        self.create_timer(self.update_period, self._publish_position)

    def _init_clients(self):
        """Initialize clients of the node."""
        self.short_motor_srv = (
            self.get_parameter("short_motor_srv").get_parameter_value().string_value
        )
        for name in self.robot_names:
            cli_motor_srv = self.create_client(
                StringReq, name + "/" + self.short_motor_srv
            )
            while not cli_motor_srv.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn(f"No position to update currently for {name}...")
            req = StringReq.Request()
            req.data = "GETODOM,1,30.0"
            future = cli_motor_srv.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            res = future.result().response.split(",")
            self.current_pos[name] = Point(
                x=float(res[1]), y=float(res[2]), z=float(res[3])
            )

    def _update_pos(self, msg, name):
        """Update the position of the robot corresponding to the name."""
        self.current_pos[name] = msg

    def _publish_position(self):
        """Publish the postion to every captor who needs information."""
        for name in self.robot_names:
            if self.current_pos[name] is not None:
                msg = PositionMap()
                msg.robot = self.current_pos[name]
                for n in [na for na in self.robot_names if na != name]:
                    msg.ennemis.append(self.current_pos[n])
                self.pub_update_position[name].publish(msg)


def main(args=None):
    """Run the main loop."""
    rclpy.init(args=args)
    position_sender_node = PositionSender()
    try:
        rclpy.spin(position_sender_node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        position_sender_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
