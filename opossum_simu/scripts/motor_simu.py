#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Simulate the state of the robot and receives the actions."""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
import time
from rclpy.executors import ExternalShutdownException
import numpy as np
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

# Import des messages
from opossum_msgs.action import StringAction
from opossum_msgs.srv import StringReq
from std_msgs.msg import String
from geometry_msgs.msg import Point
import threading
import collections


class MotorSimu(Node):
    """Simulate the state of the robot and receives the actions for navigation only."""

    def __init__(self):
        super().__init__("nav_simulation_node")
        self._init_parameters()
        self._randomize_position()
        self._init_publishers()
        self._init_action_server()
        self._init_services()
        self.get_logger().info("Nav simulation node initialized.")

    def _init_parameters(self) -> None:
        """Init parameters of the node."""
        self.declare_parameters(
            namespace="",
            parameters=[
                ("boundaries", rclpy.Parameter.Type.DOUBLE_ARRAY),
                ("real_position_topic", rclpy.Parameter.Type.STRING),
                ("short_motor_srv", rclpy.Parameter.Type.STRING),
                ("random_moves", rclpy.Parameter.Type.BOOL),
                ("compute_period", rclpy.Parameter.Type.DOUBLE),
                ("angular_velocity", rclpy.Parameter.Type.DOUBLE),
                ("linear_velocity", rclpy.Parameter.Type.DOUBLE),
            ],
        )
        self.compute_period = (
            self.get_parameter("compute_period").get_parameter_value().double_value
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
        self.blocking = False
        self.ask_for_block = False
        if self.blocking:
            self._goal_queue = collections.deque()
            self._goal_queue_lock = threading.Lock()
            self._current_goal = None
            self.handle_accepted_callback = self._queue_handle_accepted_callback
        else:
            self._goal_lock = threading.Lock()
            self._goal_handle = None
            self.handle_accepted_callback = self._single_handle_accepted_callback

    def _init_publishers(self):
        """Init publishers of the node."""
        self.real_position_topic = (
            self.get_parameter("real_position_topic").get_parameter_value().string_value
        )
        self.pub_real_position = self.create_publisher(
            Point, self.real_position_topic, 10
        )

        self.pub_zync_raspi = self.create_publisher(String, "zync_raspi", 10)
        self.timer = self.create_timer(0.1, self.send_robot_data)
    
    def send_robot_data(self):
        self.pub_zync_raspi.publish(
            String(
                data=f"ROBOTDATA {self.position[0].item()} {self.position[1].item()} {self.position[2].item()} {np.linalg.norm(self.velocity[:2]).item()} {np.arctan2(self.velocity[1], self.velocity[0]).item() - self.position[2].item()} {self.velocity[2].item()}\n"
            )
        )

    def _init_action_server(self):
        """Init server actions of the node."""
        if not self.random_moves:
            self.moveto_server = ActionServer(
                self,
                StringAction,
                "motors/action_simu",
                self.execute_callback,
                handle_accepted_callback=self.handle_accepted_callback,
                goal_callback=self.goal_callback,
                cancel_callback=self.cancel_callback,
                callback_group=ReentrantCallbackGroup(),
            )
        else:
            self.create_timer(self.compute_period, self._random_moves)

    def _init_services(self):
        """Init services of the node."""
        short_motor_srv = (
            self.get_parameter("short_motor_srv").get_parameter_value().string_value
        )
        self.srv_short_motor = self.create_service(
            StringReq,
            short_motor_srv,
            self.response_callback,
            callback_group=ReentrantCallbackGroup(),
        )

    def response_callback(self, request, response):
        """Answer to the request."""
        request_split = request.data.split(",")
        if request_split[0] == "GETODOM":
            x = str(self.position[0].item())
            y = str(self.position[1].item())
            angle = str(self.position[2].item())
            response.response = request_split[0] + "," + x + "," + y + "," + angle

        elif request_split[0] == "SPEED":
            self.linear_velocity = float(request_split[1])
            self.angular_velocity = float(request_split[3])
            response.response = request_split[0] + "," + "1"

        elif request_split[0] == "VMAX":
            self.linear_velocity = float(request_split[1])
            response.response = request_split[0] + "," + "1"

        elif request_split[0] == "VTMAX":
            self.angular_velocity = float(request_split[1])
            response.response = request_split[0] + "," + "1"

        elif request_split[0] == "MAPASSERV":
            begin = time.time()
            response.response = ""
            while time.time() - begin < 5:
                response.response += request_split[0]
                for i in range(3):
                    response.response += "," + str(self.position[i].item())
                for i in range(3):
                    response.response += "," + str(self.velocity[i].item())
                response.response += "\n"
                time.sleep(self.compute_period)

        elif request_split[0] == "BLOCK":
            self.ask_for_block = True

        return response

    def _single_handle_accepted_callback(self, goal_handle):
        """Handle recetpion of requests for only last action, no queue."""
        with self._goal_lock:
            # This server only allows one goal at a time
            if self._goal_handle is not None and self._goal_handle.is_active:
                # Abort the existing goal
                self._goal_handle.abort()
            self._goal_handle = goal_handle
        goal_handle.execute()

    def _queue_handle_accepted_callback(self, goal_handle):
        """Handle recetpion of requests queued."""
        with self._goal_queue_lock:
            if self._current_goal is not None:
                # Put incoming goal in the queue
                self._goal_queue.append(goal_handle)
            else:
                # Start goal execution right away
                self._current_goal = goal_handle
                self._current_goal.execute()

    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """Execute the actions."""
        self.ask_for_block = False
        req = goal_handle.request.data.split(",")
        name = req[0]
        args = req[1:]
        if name.upper() == "MOVE":
            self.obj = np.array(
                [
                    float(args[0]),
                    float(args[1]),
                    float(args[2]),
                ]
            )
            try:
                self.real_time = time.time()
                self.preempt_request = False
                self.current_goal_handle = goal_handle

                reached_position = np.allclose(self.position[:, 0], self.obj)

                moveto_feedback = StringAction.Feedback()
                moveto_feedback.current_state = (
                    "MOVE,"
                    + str(self.position[0].item())
                    + ","
                    + str(self.position[1].item())
                    + ","
                    + str(self.position[2].item())
                )
                while not reached_position and not self.ask_for_block:
                    if goal_handle.is_cancel_requested:
                        goal_handle.canceled()
                        return StringAction.Result()
                    if not self.blocking:
                        if not goal_handle.is_active:
                            return StringAction.Result()
                    self._ordered_moves()
                    moveto_feedback.current_state = (
                        "MOVE,"
                        + str(self.position[0].item())
                        + ","
                        + str(self.position[1].item())
                        + ","
                        + str(self.position[2].item())
                    )
                    reached_position = np.allclose(self.position[:, 0], self.obj)
                    self.real_time = time.time()
                    self.pub_real_position.publish(
                        Point(
                            x=float(self.position[0].item()),
                            y=float(self.position[1].item()),
                            z=float(self.position[2].item()),
                        )
                    )
                    time.sleep(self.compute_period)
                if self.ask_for_block:
                    self.ask_for_block = False
                    self.velocity = np.array([[0.0], [0.0], [0.0]])
                result = StringAction.Result()
                if not self.blocking:
                    with self._goal_lock:
                        if not goal_handle.is_active:
                            result.response = "Pos,done"
                            return result
                        goal_handle.succeed()
                else:
                    goal_handle.succeed()
                result.response = "Pos,done"
                return result
            finally:
                if self.blocking:
                    with self._goal_queue_lock:
                        try:
                            # Start execution of the next goal in the queue.
                            self._current_goal = self._goal_queue.popleft()
                            self._current_goal.execute()
                        except IndexError:
                            # No goal in the queue.
                            self._current_goal = None

    def _randomize_position(self):
        """Create random positions for init."""
        angle = np.random.uniform(0, 2 * np.pi)
        init_pos = np.random.uniform(0, 2, (2, 1))
        self.position = np.concatenate((init_pos, np.array([[angle]])), axis=0)
        self.velocity = np.array([[0.0], [0.0], [0.0]])
        self.acceleration = np.array([[0.0], [0.0], [0.0]])

    def _random_moves(self):
        """Do some random moves."""
        self.position[2] += np.random.uniform(-0.3, 0.3)
        pot_pos = self.position[:2, :] + np.random.uniform(-0.05, 0.05, (2, 1))
        while (
            pot_pos[0] > self.boundaries[1]
            or pot_pos[0] < self.boundaries[0]
            or pot_pos[1] > self.boundaries[3]
            or pot_pos[1] < self.boundaries[2]
        ):
            pot_pos = self.position[:2, :] + np.random.uniform(-0.05, 0.05, (2, 1))
        self.position[:2, :] = pot_pos
        self.pub_real_position.publish(
            Point(
                x=float(self.position[0].item()),
                y=float(self.position[1].item()),
                z=float(self.position[2].item()),
            )
        )

    def _ordered_moves(self):
        """Do straight moves when there is a request."""
        real_delta = time.time() - self.real_time
        deltas = self.obj[:2] - self.position[:2, 0]
        dst = np.linalg.norm(deltas)
        dangle = self.obj[2] % (2 * np.pi) - self.position[2] % (2 * np.pi)
        va = self.angular_velocity if dangle >= 0 else -self.angular_velocity
        if dst < self.linear_velocity * real_delta:
            self.velocity[:2, 0] = np.array([0, 0])
            self.position[:2, 0] = self.obj[:2]
        else:
            self.velocity[:2, 0] = self.linear_velocity * deltas / dst
            self.position[:2, 0] = (
                self.position[:2, 0] + real_delta * self.velocity[:2, 0]
            )

        if abs(real_delta * va) < abs(dangle):
            self.velocity[2, 0] = va
            self.position[2, 0] += real_delta * self.velocity[2, 0]
        else:
            self.velocity[2, 0] = 0
            self.position[2, 0] = self.obj[2]

    def destroy(self):
        """Destroy the node."""
        self.moveto_server.destroy()
        super().destroy_node()


def main(args=None):
    """Spin main loop."""
    rclpy.init(args=args)
    nav_simu_node = MotorSimu()
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(nav_simu_node, executor=executor)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        nav_simu_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
