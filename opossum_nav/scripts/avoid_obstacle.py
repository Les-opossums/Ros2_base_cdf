#!/usr/bin/env python3
"""Avoid obstacles viewed from lidar on the board."""

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
import numpy as np
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import LaserScan
from opossum_msgs.msg import RobotData, LidarLoc, GoalDetection
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker


class ObstacleAvoider(Node):
    """Avoid obstacles on the board."""

    def __init__(self) -> None:
        super().__init__("avoid_obstacle")
        self._init_parameters()
        self._init_publishers()
        self._init_subscribers()
        self.get_logger().info("Avoid obstacle node initialized.")

    def _init_parameters(self) -> None:
        """Initialize parameters."""
        self.declare_parameters(
            namespace="",
            parameters=[
                ("robot_data_topic", "robot_data"),
                ("position_topic", "position_out"),
                ("goal_position_topic", "goal_position"),
                ("scan_topic", "scan"),
                ("command_topic", "command"),
                ("visualization_topic", "visualization_limit"),
                ("obstacle_detected_topic", "obstacle_detected"),
                ("obstacle_detection_distance", 0.4),
                ("detection_mode", "full"),
                ("cone_range", 80.0),
                ("thickness", 0.2),
                ("boundaries", [0.0, 3.0, 0.0, 2.0]),
                ("boundary_limit_detection", 0.05),
                ("enable_detection", True),
                ("enable_boundary_check", False),
                ("enable_new_path", False),
                ("display_all", True),
            ],
        )
        self.display_all = (
            self.get_parameter("display_all").get_parameter_value().bool_value
        )
        self.enable_detection_default = (
            self.get_parameter("enable_detection").get_parameter_value().bool_value
        )
        self.enable_detection = self.enable_detection_default
        self.enable_boundary_check = (
            self.get_parameter("enable_boundary_check").get_parameter_value().bool_value
        )
        self.enable_new_path_default = (
            self.get_parameter("enable_new_path").get_parameter_value().bool_value
        )
        self.enable_new_path = self.enable_new_path_default
        self.obstacle_detection_distance_default = (
            self.get_parameter("obstacle_detection_distance")
            .get_parameter_value()
            .double_value
        )
        self.obstacle_detection_distance = self.obstacle_detection_distance_default
        self.detection_mode = (
            self.get_parameter("detection_mode").get_parameter_value().string_value
        )
        self.cone_range = (
            self.get_parameter("cone_range").get_parameter_value().double_value
        )
        self.thickness_default = (
            self.get_parameter("thickness").get_parameter_value().double_value
        )
        self.boundaries = (
            self.get_parameter("boundaries").get_parameter_value().double_array_value
        )
        self.boundary_limit_detection = (
            self.get_parameter("boundary_limit_detection")
            .get_parameter_value()
            .double_value
        )

        self.init_scan = False
        self.robot_data = None
        self.robot_position = None
        self.goal_position = None
        self.obstacles = None
        self.ptheta = None
        self.obstacle_detected = False
        self.in_avoid = False
        self.last_command_sent = None

    def _init_publishers(self) -> None:
        """Initialize publishers."""
        obstacle_detected_topic = (
            self.get_parameter("obstacle_detected_topic")
            .get_parameter_value()
            .string_value
        )
        command_topic = (
            self.get_parameter("command_topic").get_parameter_value().string_value
        )

        visualization_topic = (
            self.get_parameter("visualization_topic").get_parameter_value().string_value
        )

        self.pub_obstacle_detected = self.create_publisher(
            Bool, obstacle_detected_topic, 10
        )
        self.pub_command = self.create_publisher(String, command_topic, 10)

        self.pub_visualization = self.create_publisher(Marker, visualization_topic, 10)

    def _init_subscribers(self) -> None:
        """Initialize subscribers."""
        robot_data_topic = (
            self.get_parameter("robot_data_topic").get_parameter_value().string_value
        )
        position_topic = (
            self.get_parameter("position_topic").get_parameter_value().string_value
        )
        scan_topic = (
            self.get_parameter("scan_topic").get_parameter_value().string_value
        )
        goal_position_topic = (
            self.get_parameter("goal_position_topic").get_parameter_value().string_value
        )
        self.sub_robot_data = self.create_subscription(
            RobotData, robot_data_topic, self.robot_data_callback, 10
        )
        self.sub_scan = self.create_subscription(
            LaserScan, scan_topic, self.scan_callback, 10
        )
        self.sub_robot_position = self.create_subscription(
            LidarLoc, position_topic, self.robot_position_callback, 10
        )
        self.sub_goal_position = self.create_subscription(
            GoalDetection, goal_position_topic, self.goal_position_callback, 10
        )
        self.sub_au = self.create_subscription(
            Bool, "au", self.reset_all_au, 10
        )

        self.sub_end_of_match = self.create_subscription(
            Bool, "end_of_match", self.reset_all_end_of_match, 10
        )

    def reset_all_au(self, msg):
        self.get_logger().info(f"I received a stop from AU")
        if msg.data:
            self.get_logger().info(f"I received a stop from AU")
            self.enable_detection = False
            self.robot_data = None
            self.robot_position = None
            self.goal_position = None
            self.obstacles = None
            self.ptheta = None
            self.obstacle_detected = False
            self.in_avoid = False
            self.last_command_sent = None
            self._send_block()
        else:
            self.enable_detection = self.enable_detection_default
    
    def reset_all_end_of_match(self, msg):
        if msg.data:
            self.get_logger().info(f"I received a stop from END OF MATCH")
            self.enable_detection = False
            self.robot_data = None
            self.robot_position = None
            self.goal_position = None
            self.obstacles = None
            self.ptheta = None
            self.obstacle_detected = False
            self.in_avoid = False
            self.last_command_sent = None
            self._send_block()
        else:
            self.enable_detection = self.enable_detection_default


    def robot_data_callback(self, msg: RobotData) -> None:
        """Retrieve the robot datas from Zynq."""
        self.robot_data = msg

    def robot_position_callback(self, msg: LidarLoc) -> None:
        """Retrieve the robot datas from Lidar."""
        self.robot_position = msg.robot_position
        self.obstacles = msg.other_robot_position

    def goal_position_callback(self, msg) -> None:
        """Receive the goal position."""
        self.goal_position = (
            msg.goal_position
            if (msg.goal_position.x >= 0 and msg.goal_position.y >= 0)
            else None
        )
        if self.init_scan:
            self.obstacle_detected = False
            self.in_avoid = False
            if msg.detection_mode == -1:
                self.enable_detection = self.enable_detection_default
                self.enable_new_path = self.enable_new_path_default
            else:
                self.enable_detection = msg.detection_mode > 0
                self.enable_new_path = msg.detection_mode > 1
            if (
                abs(msg.obstacle_detection_distance - self.obstacle_detection_distance)
                < 0.01
            ):
                pass
            elif msg.obstacle_detection_distance >= 0:
                self.obstacle_detection_distance = msg.obstacle_detection_distance
                self._compute_security_rectangle_distances()
            else:
                self.obstacle_detection_distance = (
                    self.obstacle_detection_distance_default
                )
                self._compute_security_rectangle_distances()

    def scan_callback(self, msg: LaserScan) -> None:
        """Receive the lidar scan."""
        if not self.init_scan:
            self.init_scan = True
            self.angle_increment = msg.angle_increment
            self.index_correction = int(msg.angle_min / self.angle_increment)
            self.cone_range_index = int(
                np.pi * self.cone_range / (180 * self.angle_increment)
            )
            self.len_scan = int((msg.angle_max - msg.angle_min) / msg.angle_increment)
            self._compute_security_rectangle_distances()

        if not self.enable_detection:
            return
        if self.in_avoid:
            self.in_avoid = self._find_new_path()
        else:
            lidar_range = (
                msg.ranges[-self.index_correction :]
                + msg.ranges[: -self.index_correction]
            )
            last_obs_detected = self.obstacle_detected
            self.obstacle_detected = self.detect_obstacle(lidar_range)
            if not self.obstacle_detected and last_obs_detected != self.obstacle_detected and self.goal_position is not None:
                self._send_vmax(0.4)
                self._send_move(self.goal_position.x, self.goal_position.y, self.goal_position.z)
                return
            if self.obstacle_detected:
                if (
                    self.enable_new_path
                    and self.robot_data is not None
                    and self.robot_position is not None
                    and self.goal_position is not None
                ):
                    self._send_block()
                    self.in_avoid = self._find_new_path()
                else:
                    self._send_block()
            else:
                self.ptheta = None
        self.pub_obstacle_detected.publish(Bool(data=self.obstacle_detected))

    def detect_obstacle(self, lidar_range) -> bool:
        """Detect if there is an obstacle depending on the option."""
        if self.robot_data is None:
            return self._detect_obstacle_full(lidar_range)
        elif self.robot_data.vlin < 0.01 and not self.obstacle_detected:
            self.publish_visualization("full")
            return False
        elif self.detection_mode == "full":
            return self._detect_obstacle_full(lidar_range)
        elif self.detection_mode == "cone":
            return self._detect_obstacle_cone(lidar_range)
        elif self.detection_mode == "rectangle":
            return self._detect_obstacle_rectangle(lidar_range)
        else:
            raise ValueError("Invalid detection mode")

    def _check_in_boundaries(self, i: float, dist: float) -> bool:
        """Check if the obstacle is on the board."""
        real_position_x = self.robot_data.x + dist * np.cos(
            i * self.angle_increment + self.robot_data.theta
        )
        real_position_y = self.robot_data.y + dist * np.sin(
            i * self.angle_increment + self.robot_data.theta
        )
        return (
            real_position_x > self.boundaries[0] + self.boundary_limit_detection
            and real_position_x < self.boundaries[1] - self.boundary_limit_detection
            and real_position_y > self.boundaries[2] + self.boundary_limit_detection
            and real_position_y < self.boundaries[3] - self.boundary_limit_detection
        )

    def _detect_obstacle_full(self, lidar_range: list) -> bool:
        """Detect obstacles around robot."""
        if self.display_all:
            self.publish_visualization("full")
        try:
            if self.robot_data is None or not self.enable_boundary_check:
                for dist in lidar_range:
                    if dist < self.obstacle_detection_distance:
                        return True
                return False
            
            for i in range(len(lidar_range)):
                if lidar_range[i] < self.obstacle_detection_distance:
                    if self._check_in_boundaries(i, lidar_range[i]):
                        return True
        except Exception as e: 
            self.get_logger().info(f"Error: {e} \n" 
                                   f"len_lidar {len(lidar_range)} \n" 
                                   f"len_scan: {self.len_scan}")
        return False

    def _detect_obstacle_cone(self, lidar_range: list) -> bool:
        """Detect obstacles in a cone."""
        if self.ptheta is not None and self.robot_data.vlin < 0.001:
            theta = self.ptheta
        else:
            theta = self.robot_data.vdir
        self.ptheta = theta
        middle_angle = np.mod(theta, 2 * np.pi)
        index_middle_angle = int(middle_angle / self.angle_increment)
        if index_middle_angle - self.cone_range_index // 2 < 0:
            angle_range = list(
                range(
                    index_middle_angle - self.cone_range_index // 2 + self.len_scan,
                    self.len_scan,
                )
            ) + list(
                range(
                    0, index_middle_angle + self.cone_range_index // 2 + self.len_scan
                )
            )
        elif index_middle_angle + self.cone_range_index // 2 > self.len_scan:
            angle_range = list(
                range(index_middle_angle - self.cone_range_index // 2, self.len_scan)
            ) + list(
                range(
                    0, index_middle_angle + self.cone_range_index // 2 - self.len_scan
                )
            )
        else:
            angle_range = list(
                range(
                    index_middle_angle - self.cone_range_index // 2,
                    index_middle_angle + self.cone_range_index // 2,
                )
            )
        if self.display_all:
            self.publish_visualization("cone", angle_range)
        try:
            if not self.enable_boundary_check:
                for i in angle_range:
                    if lidar_range[i] < self.obstacle_detection_distance:
                        return True
                return False
            for i in angle_range:
                if lidar_range[i] < self.obstacle_detection_distance:
                    if self._check_in_boundaries(i, lidar_range[i]):
                        return True
        except Exception as e: 
            self.get_logger().info(f"Error: {e} \n" 
                                   f"angle_range: {angle_range[i]} \n" 
                                   f"len_lidar {len(lidar_range)} \n" 
                                   f"len_scan: {self.len_scan}")
        return False

    def _detect_obstacle_rectangle(self, lidar_range: list) -> bool:
        """Detect obstacles in the rectangle in front."""
        if self.ptheta is not None and self.robot_data.vlin < 0.001:
            theta = self.ptheta
        else:
            theta = self.robot_data.vdir
        self.ptheta = theta
        middle_angle = np.mod(theta, 2 * np.pi)
        index_middle_angle = int(middle_angle / self.angle_increment)
        if index_middle_angle - self.len_scan // 4 < 0:
            angle_range = list(
                range(3 * self.len_scan // 4 + index_middle_angle, self.len_scan)
            ) + list(range(0, index_middle_angle + self.len_scan // 4))
        elif index_middle_angle + self.len_scan // 4 > self.len_scan:
            angle_range = list(
                range(index_middle_angle - self.len_scan // 4, self.len_scan)
            ) + list(range(0, index_middle_angle - int(0.75 * self.len_scan)))
        else:
            angle_range = list(
                range(
                    index_middle_angle - self.len_scan // 4,
                    index_middle_angle + self.len_scan // 4,
                )
            )
        angle_range = angle_range[: len(self.security_dst)]
        if self.display_all:
            self.publish_visualization("rectangle", angle_range)
        try:
            if not self.enable_boundary_check:
                for i in range(len(angle_range)):
                    if lidar_range[angle_range[i]] < self.security_dst[i]:
                        return True
                return False
            for i in range(len(angle_range)):
                if lidar_range[angle_range[i]] < self.security_dst[i]:
                    if self._check_in_boundaries(
                        angle_range[i], lidar_range[angle_range[i]]
                    ):
                        return True
        except Exception as e: 
            self.get_logger().info(f"Error: {e} \n" 
                                   f"angle_range: {angle_range[i]} \n" 
                                   f"len_lidar {len(lidar_range)} \n" 
                                   f"len_scan: {self.len_scan}")
        return False

    def _compute_security_rectangle_distances(self) -> None:
        """Compute the security distance if rectangle."""
        x = self.thickness_default / 2
        y = self.obstacle_detection_distance
        theta = np.arctan(y / x)
        limit_incr = int(theta / self.angle_increment)
        self.security_dst = []
        for i in range(limit_incr):
            self.security_dst.append(x / np.cos(i * self.angle_increment))
        for i in range(limit_incr, self.len_scan // 4):
            self.security_dst.append(y / np.sin(i * self.angle_increment))
        inv_list = self.security_dst[::-1]
        self.security_dst += inv_list

    def _find_closest_obstacle(self) -> None:
        """Find the closest obstacle to avoid it."""
        if self.robot_position is None:
            return None
        min_dist = float("inf")
        closest_obstacle = None
        for obstacle in self.obstacles:
            dist = (obstacle.x - self.robot_position.x) ** 2 + (
                obstacle.y - self.robot_position.y
            ) ** 2
            if dist < min_dist and dist < (self.obstacle_detection_distance + 0.1) ** 2:
                min_dist = dist
                closest_obstacle = obstacle
        return closest_obstacle

    def _in_front(self, v1, v2) -> bool:
        """Check if the obstacle in on the trajectory."""
        return (
            v1[0] * v2[0] + v1[1] * v2[1] / (np.linalg.norm(v1) * np.linalg.norm(v2))
            > -0.2
        )

    def _obstacle_on_goal(self, obstacle) -> bool:
        """Check if the obstacle in on the goal area."""
        return (self.goal_position.x - obstacle.x) ** 2 + (
            self.goal_position.y - obstacle.y
        ) ** 2 < self.obstacle_detection_distance**2

    def _send_block(self) -> None:
        """Send block to motors."""
        # if self.last_command_sent == "BLOCK":
            # return
        cmd_msg = String()
        cmd_msg.data = "BLOCK"
        self.pub_command.publish(cmd_msg)
        self.last_command_sent = "BLOCK"

    def _send_move(self, x, y, t) -> None:
        """Send move to motors."""
        cmd_msg = String()
        cmd_msg.data = f"MOVE {x} {y} {t} 10"
        self.pub_command.publish(cmd_msg)
        self.last_command_sent = "MOVE"

    def _send_vmax(self, vmax) -> None:
        """Send move to motors."""
        cmd_msg = String()
        cmd_msg.data = f"VMAX {vmax}"
        self.pub_command.publish(cmd_msg)
        
    def _find_new_path(self) -> None:
        """Try to find a new path."""
        closest_obstacle = self._find_closest_obstacle()
        if closest_obstacle is None:
            self._send_block()
            return False
        if self._obstacle_on_goal(closest_obstacle):
            self.get_logger().info("Obstacle difficult goal we re fucked")
            # self._send_block()
            # return False
        v_rg = [
            self.goal_position.x - self.robot_position.x,
            self.goal_position.y - self.robot_position.y,
        ]
        v_ro = [
            closest_obstacle.x - self.robot_position.x,
            closest_obstacle.y - self.robot_position.y,
        ]
        if not self._is_obstacle_blocking(v_rg, v_ro, closest_obstacle, self.thickness_default / 2):
            self.get_logger().info("Path is now clear, resuming to goal.")
            self._send_move(self.goal_position.x, self.goal_position.y, self.goal_position.z)
            return False
        cross_product = 1 if v_rg[0] * v_ro[1] - v_rg[1] * v_ro[0] > 0 else -1
        pos = self._check_ways(closest_obstacle, cross_product, v_rg, v_ro)
        if pos is not None:
            self.get_logger().info(f"FOUND PATH {pos[0]}, {pos[1]}, {self.robot_data.theta}")
            self._send_vmax(0.4)
            self._send_move(pos[0], pos[1], self.robot_data.theta)
            return True
        self._send_block()
        self.get_logger().info("No way to avoid obstacle")
        return True

    def _check_ways(self, obstacle, cross, v1, v2) -> None:
        """Check if the way is possible."""
        # n is the vector to go perpendicular to the ennemi on the side of the goal
        n = [v2[1], -v2[0]]
        n = [cross * i for i in n]
        n = [i / np.linalg.norm(n) for i in n]
        limit_bound = []
        dst_bd = 0.27
        # Following lines can be reduced using 2 for loops using // 2
        # Boundary left
        k1 = (self.boundaries[0] - self.robot_position.x) / n[0]
        y_left = self.robot_position.y + k1 * n[1]
        if (
            self.boundaries[2] < y_left < self.boundaries[3]
            and abs(self.boundaries[0] - obstacle.x) > self.obstacle_detection_distance
        ):
            k2 = (self.boundaries[0] + dst_bd - self.robot_position.x) / n[0]
            y_left_2 = self.robot_position.y + k2 * n[1]
            limit_bound.append([self.boundaries[0] + dst_bd, y_left_2, 0])

        # Boundary right
        k1 = (self.boundaries[1] - self.robot_position.x) / n[0]
        y_right = self.robot_position.y + k1 * n[1]
        if (
            self.boundaries[2] < y_right < self.boundaries[3]
            and abs(self.boundaries[1] - obstacle.x) > self.obstacle_detection_distance
        ):
            k2 = (self.boundaries[1] - dst_bd - self.robot_position.x) / n[0]
            y_right_2 = self.robot_position.y + k2 * n[1]
            limit_bound.append([self.boundaries[1] - dst_bd, y_right_2, 1])

        # Boundary bottom
        k1 = (self.boundaries[2] - self.robot_position.y) / n[1]
        x_bottom = self.robot_position.x + k1 * n[0]
        if (
            self.boundaries[0] < x_bottom < self.boundaries[1]
            and abs(self.boundaries[2] - obstacle.y) > self.obstacle_detection_distance
        ):
            k2 = (self.boundaries[2] + dst_bd - self.robot_position.y) / n[1]
            x_bottom_2 = self.robot_position.x + k2 * n[0]
            limit_bound.append([x_bottom_2, self.boundaries[2] + dst_bd, 2])

        add_scene = 0.45
        # Boundary top
        k1 = (self.boundaries[3] - add_scene - self.robot_position.y) / n[1]
        x_top = self.robot_position.x + k1 * n[0]
        if (
            self.boundaries[0] < x_top < self.boundaries[1]
            and abs(self.boundaries[3] - add_scene - obstacle.y) > self.obstacle_detection_distance
        ):
            k2 = (self.boundaries[3] - add_scene - dst_bd - self.robot_position.y) / n[1]
            x_top_2 = self.robot_position.x + k2 * n[0]
            limit_bound.append([x_top_2, self.boundaries[3] - add_scene - dst_bd, 3])

        for val in limit_bound:
            v3 = [val[0] - self.robot_position.x, val[1] - self.robot_position.y]
            if (v1[0] * v3[1] - v1[1] * v3[0]) * cross < 0:
                return val
        if len(limit_bound) > 0:
            return limit_bound[0]
        return None
    
    def publish_visualization(self, mode, angle_range=None):
        marker_robot = Marker()
        marker_robot.header.frame_id = self.get_namespace() + "/laser_frame"
        marker_robot.header.stamp = self.get_clock().now().to_msg()
        marker_robot.ns = str(self.get_namespace()[1:])
        marker_robot.id = 253
        marker_robot.type = Marker.LINE_STRIP
        marker_robot.action = Marker.ADD
        marker_robot.scale.x = 0.01  # Line width
        marker_robot.color.r = 1.0
        marker_robot.color.g = 0.0
        marker_robot.color.a = 1.0
        radius = 0.1
        num_points = 100 # self.len_scan  # More points = smoother circle
        for i in range(num_points + 1):  # Close the loop
            angle = 2 * np.pi * i / num_points
            x = radius * np.cos(angle)
            y = radius * np.sin(angle)
            marker_robot.points.append(Point(x=x, y=y, z=0.0))
        self.pub_visualization.publish(marker_robot)
        
        marker = Marker()
        marker.header.frame_id = self.get_namespace() + "/laser_frame"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "limit_detection"
        marker.id = 254
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        marker.scale.x = 0.01  # Line width
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.a = 1.0
        if mode == "full":
            num_points = 100 # self.len_scan  # More points = smoother circle
            for i in range(num_points + 1):  # Close the loop
                angle = 2 * np.pi * i / num_points
                x = self.obstacle_detection_distance * np.cos(angle)
                y = self.obstacle_detection_distance * np.sin(angle)
                marker.points.append(Point(x=x, y=y, z=0.0))
        elif mode == "cone":
            marker.points.append(Point(x=0.0, y=0.0, z=0.0))
            num_points = 100
            angle_final = angle_range[-1] + 1 + self.len_scan if angle_range[-1] + 1 < angle_range[0] else angle_range[-1] + 1
            incr = (angle_final - angle_range[0]) // num_points
            for i in range(angle_range[0], angle_final, incr):
                angle = 2 * np.pi * i / self.len_scan
                x = self.obstacle_detection_distance * np.cos(angle)
                y = self.obstacle_detection_distance * np.sin(angle)
                marker.points.append(Point(x=x, y=y, z=0.0))
            marker.points.append(Point(x=0.0, y=0.0, z=0.0))
        elif mode == "rectangle":
            num_points = 100
            angle_final = angle_range[-1] + 1 + self.len_scan if angle_range[-1] + 1 < angle_range[0] else angle_range[-1] + 1
            incr = (angle_final - angle_range[0]) // num_points
            for ind, i in enumerate(range(angle_range[0], angle_final, incr)):
                angle = 2 * np.pi * i / self.len_scan
                x = self.security_dst[ind * incr] * np.cos(angle)
                y = self.security_dst[ind * incr] * np.sin(angle)
                marker.points.append(Point(x=x, y=y, z=0.0))
        self.pub_visualization.publish(marker)

    def _is_obstacle_blocking(self, v_rg, v_ro, obstacle_pos, threshold=0.3) -> bool:
        """Return True if obstacle lies close to the line from robot to goal."""
        norm_rg = np.linalg.norm(v_rg)
        if norm_rg == 0:
            return False  # Already at goal

        v_rg_unit = v_rg / norm_rg
        proj_len = np.dot(v_ro, v_rg_unit)

        if proj_len < 0:
            return False  # Obstacle is behind

        proj_point = np.array([self.robot_position.x, self.robot_position.y]) + proj_len * v_rg_unit
        obstacle_point = np.array([obstacle_pos.x, obstacle_pos.y])
        dist_to_path = np.linalg.norm(obstacle_point - proj_point)

        return dist_to_path < threshold

def main(args=None):
    """Spin main loop."""
    rclpy.init(args=args)
    node = ObstacleAvoider()
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(node, executor=executor)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
