# Import des librairies
import rclpy
from rclpy.node import Node
import copy
from .math_lidar import *
from .BeaconSorter import BeaconSorter
from .PositionFinder import PositionFinder
import time

# Import des messages
from visualization_msgs.msg import MarkerArray
from cdf_msgs.msg import Obstacles
from geometry_msgs.msg import Point
from cdf_msgs.msg import LidarLoc, MergedData
from std_msgs.msg import String
from std_srvs.srv import Trigger
from .publisher import (
    publicate_donnees_lidar,
    display_lidar_position,
    display_playground,
    diplay_fixed_beacons_positions,
    diplay_beacons_positions_found,
    display_other_robots_positions,
)

class BeaconDetectorNode(Node):

    def __init__(self):
        super().__init__("beacon_detector_node")
        self.test = True
        self._init_main_parameters()

    def _init_main_parameters(self) -> None:
        self.declare_parameters(
            namespace="",
            parameters=[
                ("enable_wait_color", False),
                ("color_topic", "init/team_color"),
                ("default_color", "yellow"),
                ("enable_robot_position_reception", False),
                ("enable_initial_position", False),
                ("distance_tolerance", 0.1),
                ("angle_tolerance", np.pi / 4),
                ("init_position", [1.62, 0.28, 0.0]),
                ("boundaries", [0.0, 2.0, 0.0, 3.0]),
                ("beacons", [0.1, 0.2, 3.1, 0.5, 1.0, 2.2, 3.4, 2.0]),
                ("object_topic", "obstacle"),
                ("robot_position_topic", "robot_position"),
                ("position_topic", "position"),
                ("debug_topic", "debug"),
                ("display_topic", "lidar_position_display"),
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
            self.get_logger().info(
                f"Team color is not enabled, default is {default_color}"
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

    def _init_parameters(self) -> None:
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
        self.fixed_beacons = []
        self.boundaries = (
            self.get_parameter("boundaries").get_parameter_value().double_array_value
        )
        beacons = self.get_parameter("beacons").get_parameter_value().double_array_value
        if self.team_color == "blue":
            for i in range(1, len(beacons), 2):
                beacons[i] = self.boundaries[3] - beacons[i]
        self.fixed_beacons = [
            make_point([beacons[i], beacons[i + 1]]) for i in range(0, len(beacons), 2)
        ]
        self.sign_vect_product = [
            get_sign_vect_product(
                self.fixed_beacons[0], self.fixed_beacons[1], self.fixed_beacons[2]
            ),
            get_sign_vect_product(
                self.fixed_beacons[0], self.fixed_beacons[1], self.fixed_beacons[3]
            ),
            get_sign_vect_product(
                self.fixed_beacons[0], self.fixed_beacons[2], self.fixed_beacons[3]
            ),
            get_sign_vect_product(
                self.fixed_beacons[1], self.fixed_beacons[2], self.fixed_beacons[3]
            ),
        ]
        self.ang_tol = (
            self.get_parameter("angle_tolerance").get_parameter_value().double_value
        )
        self.position_finder = PositionFinder(
            self.fixed_beacons,
            self.boundaries,
            self.dst_tol,
            self.init_position,
        )
        dst_beacons = {
            chr(65 + i) + chr(65 + j): dt(self.fixed_beacons[i], self.fixed_beacons[j])
            for i in range(len(self.fixed_beacons))
            for j in range(i + 1, len(self.fixed_beacons))
        }

        self.beacon_sorter = BeaconSorter(
            dst_beacons,
            self.sign_vect_product,
            self.ang_tol,
            self.dst_tol,
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
        if self.enable_robot_position_reception:
            self.robot_position_topic = (
                self.get_parameter("robot_position_topic")
                .get_parameter_value()
                .string_value
            )
            self.sub_robot_position_topic = self.create_subscription(
                MergedData,
                self.robot_position_topic,
                self.robot_position_callback,
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

    def object_callback(self, msg: Obstacles) -> None:
        if (
            not self.enable_robot_position_reception
            or self.position_finder.previous_robot is None
        ) and self.position_finder.current_robot is not None:
            self.position_finder.previous_robot = copy.deepcopy(
                self.position_finder.current_robot
            )

        previous_beacons = (
            [
                self.position_finder.current_robot.balise_A,
                self.position_finder.current_robot.balise_B,
                self.position_finder.current_robot.balise_C,
                self.position_finder.current_robot.balise_D,
            ]
            if self.position_finder.previous_robot is not None
            else None
        )
        new_objects_detected = [
            circle.center
            for circle in msg.circles
            if (circle.center.x**2 + circle.center.y**2) < 12.5
        ]

        nb_potential_beacons, potential_beacons = (
            self.beacon_sorter._find_possible_beacons(
                previous_beacons,
                new_objects_detected,
            )
        )

        position_found = self.position_finder.donnees_finales(
            nb_potential_beacons,
            potential_beacons,
            new_objects_detected,
        )

        if position_found is not None:
            msg = publicate_donnees_lidar(position_found)
            self.pub_lidar_loc.publish(msg)
            if self.test:
                self.pub_debug.publish(String(data="Position trouvée"))
                self.pub_display.publish(display_lidar_position(position_found))
                self.pub_display.publish(diplay_beacons_positions_found(position_found))
                self.pub_display.publish(display_playground(self.boundaries))
                self.pub_display.publish(
                    diplay_fixed_beacons_positions(self.fixed_beacons)
                )
                self.pub_display.publish(
                    display_other_robots_positions(position_found.other_robots)
                )

    def robot_position_callback(self, msg: Point) -> None:
        if self.position_finder.previous_robot is None:
            self.position_finder.previous_robot = RobotDatas()
            self.position_finder.previous_robot.position = msg.robots[0]
            self.position_finder.previous_robot.balise_A = chgt_base_plateau_to_robot(
                self.fixed_beacons[0], self.position_finder.previous_robot.position
            )
            self.position_finder.previous_robot.balise_B = chgt_base_plateau_to_robot(
                self.fixed_beacons[1], self.position_finder.previous_robot.position
            )
            self.position_finder.previous_robot.balise_C = chgt_base_plateau_to_robot(
                self.fixed_beacons[2], self.position_finder.previous_robot.position
            )
            self.position_finder.previous_robot.balise_D = chgt_base_plateau_to_robot(
                self.fixed_beacons[3], self.position_finder.previous_robot.position
            )
            self.position_finder.initialisation = True

def main():
    rclpy.init()
    node = BeaconDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
