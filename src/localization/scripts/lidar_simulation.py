#!/usr/bin/env python3


####Issue when sending 2 goal and the first has not ended, it fails, to publish idk why.... Need to check on actions
# Import des librairies
import rclpy
from rclpy.node import Node
import numpy as np

# Import des messages
from cdf_msgs.msg import Obstacles
from geometry_msgs.msg import Point
from cdf_msgs.msg import Obstacles, CircleObstacle
from std_srvs.srv import Trigger



class LidarSimulation(Node):

    def __init__(self):
        super().__init__("lidar_simulation_node")
        self.count = 0

        self._init_parameters()
        self._init_publishers()
        self._init_subscribers()
        self._init_services()
        self.get_logger().info("Lidar simulation node initialized.")


    def _init_parameters(self) -> None:
        self.declare_parameters(
            namespace="",
            parameters=[
                ("beacons", rclpy.Parameter.Type.DOUBLE_ARRAY),
                ("boundaries", rclpy.Parameter.Type.DOUBLE_ARRAY),
                ("object_topic", rclpy.Parameter.Type.STRING),
                ("update_position_topic", rclpy.Parameter.Type.STRING),
                ("color_service", rclpy.Parameter.Type.STRING),
                ("team_color", rclpy.Parameter.Type.STRING),
            ],
        )
        self.team_color = (
            self.get_parameter("team_color").get_parameter_value().string_value
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

    def _init_subscribers(self) -> None:
        self.update_position_topic = (
            self.get_parameter("update_position_topic").get_parameter_value().string_value
        )
        self.sub_real_position = self.create_subscription(
            Point, self.update_position_topic, self._publish_objects, 10
        )

    def _update(self, msg) -> None:
        self.position = np.array([[msg.x], [msg.y], [1]])
        self.angle = msg.z
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

        # self.new_ennemis = np.linalg.inv(self.OtoR) @ self.random_ennemi

    def _publish_objects(self, msg) -> None:
        self._update(msg)
        msg = Obstacles()
        for i in [2, 3 ,0, 1]:
            self.get_logger().info(f"ind {i}: x={self.new_beacons[i][0, 0]}, y={self.new_beacons[i][1, 0]}")
            circle = CircleObstacle()
            circle.center.x = self.new_beacons[i][0, 0]
            circle.center.y = self.new_beacons[i][1, 0]
            circle.radius = 0.1
            msg.circles.append(circle)
        # circle = CircleObstacle()
        # circle.center.x = self.new_ennemis[0, 0]
        # circle.center.y = self.new_ennemis[1, 0]
        # circle.radius = 0.1
        # msg.circles.append(circle)
        self.pub_object.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    lidar_simulation_node = LidarSimulation()
    try:
        rclpy.spin(lidar_simulation_node)
    except:
        pass
    finally:
        lidar_simulation_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()