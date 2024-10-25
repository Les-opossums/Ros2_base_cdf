# Import des librairies
import rclpy
from rclpy.node import Node
import time
import numpy as np
import matplotlib.pyplot as plt

# Import des messages
from cdf_msgs.msg import Obstacles
from geometry_msgs.msg import Point
from cdf_msgs.msg import Obstacles, CircleObstacle
from std_srvs.srv import Trigger


class BeaconGeneratorNode(Node):

    def __init__(self):
        super().__init__("beacon_generator_node")
        self._init_parameters()
        self._init_publishers()
        self._init_services()
        self._randomize_position()
        self.create_timer(1, self._publish_objects)

    def _init_parameters(self) -> None:
        self.declare_parameters(
            namespace="",
            parameters=[
                ("beacons", [0.1, 0.2, 3.1, 0.5, 1.0, 2.2, 3.4, 2.0]),
                ("boundaries", [0.0, 2.0, 0.0, 3.0]),
                ("object_topic", "obstacle"),
                ("color_service", "init/team_color"),
                ("team_color", "blue"),
            ],
        )
        self.boundaries = (
            self.get_parameter("boundaries").get_parameter_value().double_array_value
        )
        self.team_color = (
            self.get_parameter("team_color").get_parameter_value().string_value
        )
        if self.team_color not in ["blue", "yellow"]:
            raise ValueError("Invalid team color")
        beacons = self.get_parameter("beacons").get_parameter_value().double_array_value
        if self.team_color == "blue":
            for i in range(1, len(beacons), 2):
                beacons[i] = self.boundaries[3] - beacons[i]
        self.fixed_beacons = [
            np.array([beacons[i], beacons[i + 1], 1]).reshape(3, 1)
            for i in range(0, len(beacons), 2)
        ]

    def _init_services(self) -> None:
        self.color_service_name = (
            self.get_parameter("color_service").get_parameter_value().string_value
        )
        self.color_service = self.create_service(
            Trigger, self.color_service_name, self._set_color
        )

    def _set_color(self, request, response) -> Trigger.Response:
        response.success = True
        response.message = f"{self.team_color}"
        return response

    def _init_publishers(self) -> None:
        self.object_topic = (
            self.get_parameter("object_topic").get_parameter_value().string_value
        )
        self.pub_object = self.create_publisher(Obstacles, self.object_topic, 10)

    def _randomize_position(self) -> Point:
        self.angle = 5 # np.random.uniform(0, 2 * np.pi)
        self.position = np.random.uniform(0, 2, (3, 1))
        self.random_ennemi = np.random.uniform(0, 2, (3, 1))
        self.position[2, 0] = 1
        self.random_ennemi[2, 0] = 1
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
        self.new_ennemis = np.linalg.inv(self.OtoR) @ self.random_ennemi

    def _update_obstacles(self) -> None:
        self.angle +=  0.2 # np.random.uniform(-0.1, 0.1)
        self.position[:2, :] += np.random.uniform(-0.01, 0.01, (2, 1))
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
        self.new_ennemis = np.linalg.inv(self.OtoR) @ self.random_ennemi

    def _publish_objects(self) -> None:
        self._update_obstacles()
        msg = Obstacles()
        for beacon in self.new_beacons:
            circle = CircleObstacle()
            circle.center.x = beacon[0, 0]
            circle.center.y = beacon[1, 0]
            circle.radius = 0.1
            msg.circles.append(circle)
        circle = CircleObstacle()
        circle.center.x = self.new_ennemis[0, 0]
        circle.center.y = self.new_ennemis[1, 0]
        circle.radius = 0.1
        msg.circles.append(circle)
        self.pub_object.publish(msg)

    def plot_positions(self):
        plt.xlim(-6, 6)
        plt.ylim(-6, 6)
        plt.plot(
            [beacon[0, 0] for beacon in self.fixed_beacons],
            [beacon[1, 0] for beacon in self.fixed_beacons],
            "bo",
        )
        print([beacon[0, 0] for beacon in self.new_beacons])
        print([beacon[1, 0] for beacon in self.new_beacons])
        plt.plot(
            [beacon[0, 0] for beacon in self.new_beacons],
            [beacon[1, 0] for beacon in self.new_beacons],
            "ro",
        )
        plt.show()

def main():
    rclpy.init()
    node = BeaconGeneratorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
