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
from std_msgs.msg import String, Int16
from geometry_msgs.msg import Point
import threading
import collections


class ActuatorsSimu(Node):
    """Simulate the state of the robot and receives the actions for navigation only."""

    def __init__(self):
        super().__init__("nav_simulation_node")
        self._init_parameters()
        self._init_publishers()
        self._init_action_server()
        self._init_services()
        self._init_states()
        self.get_logger().info("Actuator simulation node initialized.")

    def _init_parameters(self) -> None:
        """Init parameters of the node."""
        # self.declare_parameters(
        #     namespace="",
        #     parameters=[
        #         ("short_motor_srv", rclpy.Parameter.Type.STRING),
        #     ],
        # )
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
        """Init publishers of the node."""
        self.pub_change_state = self.create_publisher(
            Int16, "actuators_state", 10
        )
        self.pub_zync_raspi = self.create_publisher(String, "zync_raspi", 10)
    
    def _init_action_server(self):
        """Init server actions of the node."""
        self.actuators_server = ActionServer(
            self,
            StringAction,
            "actuators/action_simu",
            self.execute_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup(),
        )

    def _init_services(self):
        """Init services of the node."""
        self.create_service(
            StringReq,
            "actuators/service_simu",
            self.response_callback,
            callback_group=ReentrantCallbackGroup(),
        )

    def response_callback(self, request, response):
        """Answer to the request."""
        request_split = request.data.split(",")
        if request_split[0] == "PUMP":
            self.states[f"PUMP{request_split[1]}"] = bool(int(request_split[2]))
            response.response = request_split[0] + "," + request_split[1] + "," + request_split[2]
        state_vector = encode_state(list(self.states.values()))
        self.pub_change_state.publish(Int16(data=state_vector))
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
        req = goal_handle.request.data.split(",")
        name = req[0]
        args = req[1:]
        pass

    def _init_states(self):
        """Create random positions for init."""
        self.states = {f"PUMP{i}": False for i in range(4)}
        state_vector = encode_state(list(self.states.values()))
        self.pub_change_state.publish(Int16(data=state_vector))

    def destroy(self):
        """Destroy the node."""
        self.actuators_server.destroy()
        super().destroy_node()

def encode_state(state) -> int:
    """Encode a list of booleans into an integer."""
    result = 0
    for i, bit in enumerate(state):
        if bit:
            result |= (1 << i)
    return result

def decode_state(self, value: int, length: int) -> list[bool]:
    """Decode an integer into a list of booleans of given length."""
    return [(value >> i) & 1 == 1 for i in range(length)]


def main(args=None):
    """Spin main loop."""
    rclpy.init(args=args)
    actuators_simu_node = ActuatorsSimu()
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(actuators_simu_node, executor=executor)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        actuators_simu_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
