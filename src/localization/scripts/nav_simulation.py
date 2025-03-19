#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
import time
import numpy as np
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

# Import des messages
from cdf_msgs.action import MoveTo
from cdf_msgs.srv import PosTrigger
from geometry_msgs.msg import Point
import threading
import collections

class NavSimulation(Node):

    def __init__(self):
        super().__init__("nav_simulation_node")
        self._init_parameters()
        self._randomize_position()
        self._init_publishers()
        self._init_server_actions()
        self._init_services()
        self.get_logger().info(f"Nav simulation node initialized.")


    def _init_parameters(self) -> None:
        self.declare_parameters(
            namespace="",
            parameters=[
                ("boundaries", rclpy.Parameter.Type.DOUBLE_ARRAY),
                ("real_position_topic", rclpy.Parameter.Type.STRING),
                ("trigger_position_srv", rclpy.Parameter.Type.STRING),
                ("random_moves", rclpy.Parameter.Type.BOOL),
                ("moveto_action", rclpy.Parameter.Type.STRING),
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
        self.real_position_topic = (
            self.get_parameter("real_position_topic").get_parameter_value().string_value
        )
        self.pub_real_position = self.create_publisher(Point, self.real_position_topic, 10)

    def _init_server_actions(self):
        if not self.random_moves:
            self.moveto_action = (
                self.get_parameter("moveto_action").get_parameter_value().string_value
            )
            self.get_logger().info(f"REceiving on {self.moveto_action}")
            self.get_logger().info(f"Should receive on {self.get_namespace()}")
            self.moveto_server = ActionServer(
                self,
                MoveTo, 
                self.get_namespace() + '/' + self.moveto_action,
                self.execute_callback,
                handle_accepted_callback=self.handle_accepted_callback,
                goal_callback=self.goal_callback, 
                cancel_callback=self.cancel_callback,
                callback_group=ReentrantCallbackGroup()
            )
        else:
            self.create_timer(self.compute_period, self._random_moves)

    def _init_services(self):
        self.trigger_position_srv = (
                self.get_parameter("trigger_position_srv").get_parameter_value().string_value
            )
        self.srv_trigger_position = self.create_service(PosTrigger, self.trigger_position_srv, self._send_position)

    def _send_position(self, request, response):
        response.pos.x = float(self.position[0])
        response.pos.y = float(self.position[1])
        response.pos.z = float(self.angle)
        return response
    
    def _single_handle_accepted_callback(self, goal_handle):
        self.get_logger().info(f"Im here")
        with self._goal_lock:
            # This server only allows one goal at a time
            if self._goal_handle is not None and self._goal_handle.is_active:
                # Abort the existing goal
                self._goal_handle.abort()
            self._goal_handle = goal_handle

        goal_handle.execute()

    def _queue_handle_accepted_callback(self, goal_handle):
        """Start or defer execution of an already accepted goal."""
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
    
    def destroy(self):
        self.moveto_server.destroy()
        super().destroy_node()
    
    def execute_callback(self, goal_handle):
        self.get_logger().info(f"Im executing now")
        try:
            self.real_time = time.time()
            self.preempt_request = False
            self.current_goal_handle = goal_handle
            reached_position = (self.position[0] == goal_handle.request.goal.x and
                                self.position[1] == goal_handle.request.goal.y and 
                                self.angle == goal_handle.request.goal.z) 
            self.obj = np.array([goal_handle.request.goal.x, goal_handle.request.goal.y, goal_handle.request.goal.z])
            moveto_feedback = MoveTo.Feedback()
            moveto_feedback.current_position.x = float(self.position[0])
            moveto_feedback.current_position.y = float(self.position[1])
            moveto_feedback.current_position.z = float(self.angle)
            while not reached_position:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    return MoveTo.Result()
                if not self.blocking:
                    if not goal_handle.is_active:
                        return MoveTo.Result()
                self._ordered_moves()
                moveto_feedback.current_position.x = float(self.position[0])
                moveto_feedback.current_position.y = float(self.position[1])
                moveto_feedback.current_position.z = float(self.angle)
                reached_position = (self.position[0] == goal_handle.request.goal.x and
                                    self.position[1] == goal_handle.request.goal.y and 
                                    self.angle == goal_handle.request.goal.z) 
                # goal_handle.publish_feedback(moveto_feedback)
                self.real_time = time.time()
                self.pub_real_position.publish(moveto_feedback.current_position)
                time.sleep(self.compute_period)
            if not self.blocking:
                with self._goal_lock:
                    if not goal_handle.is_active:
                        return MoveTo.Result()
                    goal_handle.succeed()
            else:
                goal_handle.succeed()
            result = MoveTo.Result()
            result.result.x = moveto_feedback.current_position.x
            result.result.y = moveto_feedback.current_position.y
            result.result.z = moveto_feedback.current_position.z
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
    
    def _randomize_position(self) -> Point:
        self.angle = np.random.uniform(0, 2 * np.pi)
        self.position = np.random.uniform(0, 2, (2, 1))
    
    def _random_moves(self):
        self.angle +=  np.random.uniform(-0.3, 0.3)
        pot_pos = self.position + np.random.uniform(-0.05, 0.05, (2, 1))
        while (pot_pos[0] > self.boundaries[1] or pot_pos[0] < self.boundaries[0]
            or pot_pos[1] > self.boundaries[3] or pot_pos[1] < self.boundaries[2]):
            pot_pos = self.position[:2, :] + np.random.uniform(-0.05, 0.05, (2, 1))
        self.position[:2,  :] = pot_pos

    def _ordered_moves(self):
        real_delta = time.time() - self.real_time
        deltax = self.obj[0] - self.position[0]
        deltay = self.obj[1] - self.position[1]
        sqrt = np.sqrt(deltax ** 2 + deltay ** 2)
        vx, vy = self.linear_velocity * deltax / sqrt, self.linear_velocity * deltay / sqrt
        dangle = self.obj[2] % (2 * np.pi) - self.angle % (2 * np.pi)
        va = self.angular_velocity if dangle >= 0 else -self.angular_velocity
        self.position[0] = (self.position[0] + real_delta * vx 
            if abs(real_delta * vx) < abs(deltax)
            else self.obj[0])
        self.position[1] = (self.position[1] + real_delta * vy 
            if abs(real_delta * vy) < abs(deltay)
            else self.obj[1])
        self.angle = (self.angle + real_delta * va 
            if abs(real_delta * va) < abs(dangle)
            else self.obj[2])

def main(args=None):
    rclpy.init(args=args)
    nav_simu_node = NavSimulation()
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(nav_simu_node, executor=executor)
    except Exception:
        pass
    finally:
        nav_simu_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()