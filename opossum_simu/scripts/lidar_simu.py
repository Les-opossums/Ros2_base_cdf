#!/usr/bin/env python3
"""Simulate the lidar sensor, outputs the beacons in the robot frame."""


# Issue when sending 2 goal and the first has not ended, it fails, to publish idk why.... Need to check on actions
# Import des librairies
import rclpy
from rclpy.node import Node
import numpy as np

# Import des messages
from rclpy.executors import ExternalShutdownException
from opossum_msgs.msg import PositionMap
from obstacle_detector.msg import Obstacles, CircleObstacle
from std_srvs.srv import Trigger
from std_msgs.msg import Header, String
from opossum_simu.math_simu import lidar_scan
from sensor_msgs.msg import LaserScan


class LidarSimulation(Node):
    """Simulate the lidar sensor by returning points corresponding to beacons."""

    def __init__(self):
        super().__init__("lidar_simulation_node")
        self.declare_parameters(
            namespace="",
            parameters=[
                ("beacons", rclpy.Parameter.Type.DOUBLE_ARRAY),
                ("boundaries", rclpy.Parameter.Type.DOUBLE_ARRAY),
                ("object_topic", rclpy.Parameter.Type.STRING),
                ("scan_topic", rclpy.Parameter.Type.STRING),
                ("update_position_topic", rclpy.Parameter.Type.STRING),
                ("color_service", rclpy.Parameter.Type.STRING),
                ("color_topic", rclpy.Parameter.Type.STRING),
                ("default_color", rclpy.Parameter.Type.STRING),
                ("available_colors", rclpy.Parameter.Type.STRING_ARRAY),
                ("use_lidar_points", rclpy.Parameter.Type.BOOL),
                ("enable_wait_color", rclpy.Parameter.Type.BOOL),
                ("radius", rclpy.Parameter.Type.DOUBLE),
                ("lidar_range", rclpy.Parameter.Type.DOUBLE),
                ("angle_info", rclpy.Parameter.Type.DOUBLE_ARRAY),
            ],
        )
        self._init_main_parameters()

    def _init_main_parameters(self: Node) -> None:
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
                self.color_topic,
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

    
    def _init_parameters(self) -> None:
        self.boundaries = (
            self.get_parameter("boundaries").get_parameter_value().double_array_value
        )
        self.radius = self.get_parameter("radius").get_parameter_value().double_value
        self.use_lidar_points = (
            self.get_parameter("use_lidar_points").get_parameter_value().bool_value
        )
        self.lidar_range = (
            self.get_parameter("lidar_range").get_parameter_value().double_value
        )
        self.angle_info = (
            self.get_parameter("angle_info").get_parameter_value().double_array_value
        )
        beacons = self.get_parameter("beacons").get_parameter_value().double_array_value
        if self.team_color not in self.available_colors:
            raise ValueError("Invalid team color")
        if self.team_color == "blue":
            for i in range(0, len(beacons), 2):
                beacons[i] = self.boundaries[1] - beacons[i]
        self.fixed_beacons = [
            np.array([beacons[i], beacons[i + 1], 1]).reshape(3, 1)
            for i in range(0, len(beacons), 2)
        ]
        self.get_logger().info(f"Lidar simulation node initialized with color {self.team_color}.")

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
        if self.use_lidar_points:
            self.scan_topic = (
                self.get_parameter("scan_topic").get_parameter_value().string_value
            )
            self.pub_scan = self.create_publisher(LaserScan, self.scan_topic, 10)
        else:
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
        """Update the position and transform fixed beacons and enemy positions to the robot frame."""
        x, y, theta = msg.robot.x, msg.robot.y, msg.robot.z
        self.position = np.array([[x], [y], [1]])
        self.angle = theta

        # Homogeneous transformation matrix: world → robot frame
        cos_a = np.cos(theta)
        sin_a = np.sin(theta)
        self.OtoR = np.array([
            [cos_a, -sin_a, x],
            [sin_a,  cos_a, y],
            [0, 0, 1]
        ])

        inv_OtoR = np.array([
            [ cos_a,  sin_a, -x * cos_a - y * sin_a],
            [-sin_a,  cos_a,  x * sin_a - y * cos_a],
            [0, 0, 1]
        ])

        # Transform all beacons
        self.new_beacons = [inv_OtoR @ b for b in self.fixed_beacons]

        # Vectorized transformation of enemies
        self.new_ennemis = []
        if msg.ennemis:
            ennemis_world = np.array([[e.x, e.y, 1.0] for e in msg.ennemis]).T  # Shape: (3, N)
            ennemis_robot = inv_OtoR @ ennemis_world
            self.new_ennemis = [ennemis_robot[:, i].reshape(3, 1) for i in range(ennemis_robot.shape[1])]

    # def _update(self, msg) -> None:
    #     """Update the position and beacons in robot frame."""
    #     self.position = np.array([[msg.robot.x], [msg.robot.y], [1]])
    #     self.angle = msg.robot.z
    #     cos_a = np.cos(self.angle)
    #     sin_a = np.sin(self.angle)
    #     self.OtoR = np.array(
    #         [
    #             [cos_a, -sin_a, self.position[0, 0]],
    #             [sin_a, cos_a, self.position[1, 0]],
    #             [0, 0, 1],
    #         ]
    #     )
    #     inv_OtoR = np.linalg.inv(self.OtoR)
    #     self.new_beacons = [
    #         inv_OtoR @ beacon for beacon in self.fixed_beacons
    #     ]
    #     self.new_ennemis = []
    #     for e in msg.ennemis:
    #         e_w = np.array([[e.x], [e.y], [1]])
    #         self.new_ennemis.append(inv_OtoR @ e_w)

    def _publish_objects(self, msg) -> None:
        """Publish all the objects displayed."""
        self._update(msg)
        if self.use_lidar_points:
            msg = LaserScan()
            objects = []
            for beacon in self.new_beacons:
                objects.append(
                    {"type": "circle", "center": beacon[:2, 0], "radius": self.radius}
                )
            for e in self.new_ennemis:
                objects.append({"type": "circle", "center": e[:2, 0], "radius": 0.1})
            scan_result = lidar_scan(
                self.angle_info[0],
                self.angle_info[1],
                self.angle_info[2],
                objects,
                self.lidar_range,
            )
            now = self.get_clock().now().to_msg()
            msg.header = Header(
                stamp=now, frame_id=self.get_namespace()[1:] + "/laser_frame"
            )
            msg.angle_min = self.angle_info[0]
            msg.angle_max = self.angle_info[1]
            msg.angle_increment = self.angle_info[2]
            msg.time_increment = 0.0
            msg.scan_time = 0.1
            msg.range_max = self.lidar_range
            msg.range_min = 0.12
            msg.ranges = scan_result
            intensities = np.random.uniform(0, 255, len(scan_result))
            msg.intensities = intensities.tolist()
            self.pub_scan.publish(msg)
        else:
            msg = Obstacles()
            for beacon in self.new_beacons:
                circle = CircleObstacle()
                circle.center.x = beacon[0, 0]
                circle.center.y = beacon[1, 0]
                circle.radius = self.radius
                msg.circles.append(circle)

            for e in self.new_ennemis:
                circle = CircleObstacle()
                circle.center.x = e[0, 0]
                circle.center.y = e[1, 0]
                circle.radius = self.radius
                msg.circles.append(circle)
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
