#!/usr/bin/env python3
"""Simulate the lidar sensor, outputs the beacons in the robot frame."""


# Issue when sending 2 goal and the first has not ended, it fails, to publish idk why.... Need to check on actions
# Import des librairies
import rclpy
from rclpy.node import Node
import numpy as np

# Import des messages
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Point
from cdf_msgs.msg import Obstacles, CircleObstacle, PositionMap
from std_srvs.srv import Trigger


class LidarSimulation(Node):
    """Simulate the lidar sensor by returning points corresponding to beacons."""

    def __init__(self):
        super().__init__("lidar_simulation_node")
        self.count = 0

        self._init_parameters()
        self._init_publishers()
        self._init_subscribers()
        self._init_services()
        self.get_logger().info("Lidar simulation node initialized.")

    def _init_parameters(self) -> None:
        """Initialize parameters."""
        self.declare_parameters(
            namespace="",
            parameters=[
                ("beacons", rclpy.Parameter.Type.DOUBLE_ARRAY),
                ("boundaries", rclpy.Parameter.Type.DOUBLE_ARRAY),
                ("object_topic", rclpy.Parameter.Type.STRING),
                ("update_position_topic", rclpy.Parameter.Type.STRING),
                ("color_service", rclpy.Parameter.Type.STRING),
                ("default_color", rclpy.Parameter.Type.STRING),
            ],
        )
        self.team_color = (
            self.get_parameter("default_color").get_parameter_value().string_value
        )
        self.boundaries = (
            self.get_parameter("boundaries").get_parameter_value().double_array_value
        )
        beacons = self.get_parameter("beacons").get_parameter_value().double_array_value
        if self.team_color not in ["blue", "yellow"]:
            raise ValueError("Invalid team color")
        if self.team_color == "blue":
            for i in range(1, len(beacons), 2):
                beacons[i] = self.boundaries[3] - beacons[i]
        self.fixed_beacons = [
            np.array([beacons[i], beacons[i + 1], 1]).reshape(3, 1)
            for i in range(0, len(beacons), 2)
        ]

    def _init_services(self) -> None:
        """Initialize services."""
        self.color_service_name = (
            self.get_parameter("color_service").get_parameter_value().string_value
        )
        self.color_service = self.create_service(
            Trigger, self.color_service_name, self._set_color
        )

    def _set_color(self, request, response) -> Trigger.Response:
        """Set team color."""
        response.success = True
        response.message = f"{self.team_color}"
        return response

    def _init_publishers(self) -> None:
        """Initialize publishers."""
        self.object_topic = (
            self.get_parameter("object_topic").get_parameter_value().string_value
        )
        self.pub_object = self.create_publisher(Obstacles, self.object_topic, 10)

    def _init_subscribers(self) -> None:
        """Initialize subscribers."""
        self.update_position_topic = (
            self.get_parameter("update_position_topic")
            .get_parameter_value()
            .string_value
        )
        self.sub_real_position = self.create_subscription(
            PositionMap, self.update_position_topic, self._publish_objects, 10
        )

    def _update(self, msg) -> None:
        """Update the position and beacons in robot frame."""
        self.position = np.array([[msg.robot.x], [msg.robot.y], [1]])
        self.angle = msg.robot.z
        self.OtoR = np.array(
            [
                [np.cos(self.angle), -np.sin(self.angle), self.position[0, 0]],
                [np.sin(self.angle), np.cos(self.angle), self.position[1, 0]],
                [0, 0, 1],
            ]
        )
        self.new_beacons = [
            np.linalg.inv(self.OtoR) @ beacon for beacon in self.fixed_beacons
        ]

        self.new_ennemis = []
        for e in msg.ennemis:
            e_w = np.array([[e.x], [e.y], [1]])
            self.new_ennemis.append(np.linalg.inv(self.OtoR) @ e_w)

    def _publish_objects(self, msg) -> None:
        """Publish all the objects displayed."""
        self._update(msg)
        msg = Obstacles()
        for i in [2, 3, 0]:
            # self.get_logger().info(
            #     f"ind {i}: x={self.new_beacons[i][0, 0]}, y={self.new_beacons[i][1, 0]}"
            # )
            circle = CircleObstacle()
            circle.center.x = self.new_beacons[i][0, 0]
            circle.center.y = self.new_beacons[i][1, 0]
            circle.radius = 0.1
            msg.circles.append(circle)

        for e in self.new_ennemis:
            circle = CircleObstacle()
            circle.center.x = e[0, 0]
            circle.center.y = e[1, 0]
            circle.radius = 0.1
            msg.circles.append(circle)
        # self.get_logger().info(
        #     f"x: {self.position[0]}, y: {self.position[1]}: z:{self.angle}"
        # )
        # circle = CircleObstacle()
        # circle.radius = 0.1
        # msg.circles.append(circle)
        self.pub_object.publish(msg)


def main(args=None):
    """Run main loop."""
    rclpy.init(args=args)
    lidar_simulation_node = LidarSimulation()
    try:
        rclpy.spin(lidar_simulation_node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        lidar_simulation_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
