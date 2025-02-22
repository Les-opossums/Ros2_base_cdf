#!/usr/bin/env python3


####Issue when sending 2 goal and the first has not ended, it fails, to publish idk why.... Need to check on actions
# Import des librairies
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
import time
import numpy as np
import matplotlib.pyplot as plt
import threading
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

# Import des messages
from cdf_msgs.msg import Obstacles
from cdf_msgs.action import MoveTo
from geometry_msgs.msg import Point
from cdf_msgs.msg import Obstacles, CircleObstacle
from std_srvs.srv import Trigger


class BeaconGeneratorNode(Node):

    def __init__(self):
        super().__init__("beacon_generator_node")
        self.count = 0

        self._init_parameters()
        self._init_publishers()
        self._init_services()
        self._randomize_position()
        self._init_server_actions()

    def _init_parameters(self) -> None:
        self.declare_parameters(
            namespace="",
            parameters=[
                ("beacons", rclpy.Parameter.Type.DOUBLE_ARRAY),
                ("boundaries", rclpy.Parameter.Type.DOUBLE_ARRAY),
                ("object_topic", rclpy.Parameter.Type.STRING),
                ("real_position_topic", rclpy.Parameter.Type.STRING),
                ("color_service", rclpy.Parameter.Type.STRING),
                ("team_color", rclpy.Parameter.Type.STRING),
                ("random_moves", rclpy.Parameter.Type.BOOL),
                ("moveto_action", rclpy.Parameter.Type.STRING),
                ("period", rclpy.Parameter.Type.DOUBLE),
                ("angular_velocity", rclpy.Parameter.Type.DOUBLE),
                ("linear_velocity", rclpy.Parameter.Type.DOUBLE),
            ],
        )
        self.team_color = (
            self.get_parameter("team_color").get_parameter_value().string_value
        )
        self.period = (
            self.get_parameter("period").get_parameter_value().double_value
        )
        self.angular_velocity = (
            self.get_parameter("angular_velocity").get_parameter_value().double_value
        )
        self.linear_velocity = (
            self.get_parameter("linear_velocity").get_parameter_value().double_value
        )
        self.random_moves = (
            self.get_parameter("random_moves").get_parameter_value().bool_value
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
    def _init_server_actions(self):
        timer_callback_group = ReentrantCallbackGroup()
        move_callback_group = timer_callback_group
        self.period = 0.5
        if not self.random_moves:
            self.moveto_action = (
                self.get_parameter("moveto_action").get_parameter_value().string_value
            )
            self.moveto_server = ActionServer(
                self, 
                MoveTo, 
                self.moveto_action,
                self.moveto_callback,
                callback_group=move_callback_group
            )
        self.create_timer(self.period, self._publish_objects, callback_group=timer_callback_group)

    def moveto_callback(self, goal_handle):
        self.get_logger().info("Executing Goal... ")
        # rate.sleep()
        reached_position = (self.position[0] == goal_handle.request.goal.x and
                            self.position[1] == goal_handle.request.goal.y and 
                            self.angle == goal_handle.request.goal.z) 
        self.obj = np.array([goal_handle.request.goal.x, goal_handle.request.goal.y, goal_handle.request.goal.z])
        moveto_feedback = MoveTo.Feedback()
        moveto_feedback.current_position.x = float(self.position[0])
        moveto_feedback.current_position.y = float(self.position[1])
        moveto_feedback.current_position.z = float(self.angle)
        while not reached_position:
            time.sleep(self.period)
            self._update()
            moveto_feedback.current_position.x = float(self.position[0])
            moveto_feedback.current_position.y = float(self.position[1])
            moveto_feedback.current_position.z = float(self.angle)
            reached_position = (self.position[0] == goal_handle.request.goal.x and
                                self.position[1] == goal_handle.request.goal.y and 
                                self.angle == goal_handle.request.goal.z) 
            goal_handle.publish_feedback(moveto_feedback)
        self.get_logger().info("Done Goal... ")
        goal_handle.succeed()
        result = MoveTo.Result()
        result.result.x = moveto_feedback.current_position.x
        result.result.y = moveto_feedback.current_position.y
        result.result.z = moveto_feedback.current_position.z
        return result
    
    def _set_color(self, request, response) -> Trigger.Response:
        response.success = True
        response.message = f"{self.team_color}"
        return response

    def _init_publishers(self) -> None:
        self.object_topic = (
            self.get_parameter("object_topic").get_parameter_value().string_value
        )
        self.pub_object = self.create_publisher(Obstacles, self.object_topic, 10)
        self.real_position_topic = (
            self.get_parameter("real_position_topic").get_parameter_value().string_value
        )
        self.pub_real_position = self.create_publisher(Point, self.real_position_topic, 10)

    def _randomize_position(self) -> Point:
        self.angle = np.random.uniform(0, 2 * np.pi)
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
        self._moves = self._random_moves if self.random_moves else self._ordered_moves
    
    def _random_moves(self):
        self.angle +=  np.random.uniform(-0.3, 0.3)
        pot_pos = self.position[:2, :] + np.random.uniform(-0.05, 0.05, (2, 1))
        while (pot_pos[0] > self.boundaries[1] or pot_pos[0] < self.boundaries[0]
            or pot_pos[1] > self.boundaries[3] or pot_pos[1] < self.boundaries[2]):
            pot_pos = self.position[:2, :] + np.random.uniform(-0.05, 0.05, (2, 1))
        self.position[:2,  :] = pot_pos

    def _ordered_moves(self):
        deltax = self.obj[0] - self.position[0]
        deltay = self.obj[1] - self.position[1]
        sqrt = np.sqrt(deltax ** 2 + deltay ** 2)
        vx, vy = self.linear_velocity * deltax / sqrt, self.linear_velocity * deltay / sqrt
        dangle = self.obj[2] % (2 * np.pi) - self.angle % (2 * np.pi)
        va = self.angular_velocity if dangle >= 0 else -self.angular_velocity
        self.position[0] = (self.position[0] + self.period * vx 
            if abs(self.period * vx) < abs(deltax)
            else self.obj[0])
        self.position[1] = (self.position[1] + self.period * vy 
            if abs(self.period * vy) < abs(deltay)
            else self.obj[1])
        self.angle = (self.angle + self.period * va 
            if abs(self.period * va) < abs(dangle)
            else self.obj[2])
        
    def _update(self) -> None:
        self._moves()
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
        if self.random_moves:
            self._update()
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
        pos_msg = Point()
        pos_msg.x = float(self.position[0])
        pos_msg.y = float(self.position[1])
        pos_msg.z = float(self.angle)
        self.count += 1
        self.get_logger().info(f'Im puclishing rn {self.count}')
        
        self.pub_real_position.publish(pos_msg)
        # thread_plot = threading.Thread(target=self.plot_positions)
        # thread_plot.start()

    def plot_positions(self):
        # plt.xlim(-6, 6)
        # plt.ylim(-6, 6)
        plt.xlim(0, 2)
        plt.ylim(0, 3)
        # plt.plot(
        #     [beacon[0, 0] for beacon in self.fixed_beacons],
        #     [beacon[1, 0] for beacon in self.fixed_beacons],
        #     "bo",
        #     # label='Fixed Beacons',
        # )
        # print([beacon[0, 0] for beacon in self.new_beacons])
        # print([beacon[1, 0] for beacon in self.new_beacons])
        # plt.plot(
        #     [beacon[0, 0] for beacon in self.new_beacons],
        #     [beacon[1, 0] for beacon in self.new_beacons],
        #     "ro",
        #     # label='New Beacons'
        # )
        plt.plot(
            self.position[0],
            self.position[1],
            "go",
            # label='Current Position'
        )
        ax = plt.gca()
        ax.set_aspect('equal', adjustable='box')
        # plt.legend()
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    beacon_generator_node = BeaconGeneratorNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(beacon_generator_node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        beacon_generator_node.get_logger().info('Keyboard interrupt, shutting down.\n')
    beacon_generator_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()