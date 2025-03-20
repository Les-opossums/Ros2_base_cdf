#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

# Import des messages
from cdf_msgs.action import ActionSequencer
from cdf_msgs.srv import PosTrigger
from geometry_msgs.msg import Point
import threading
import collections


class ActionManager(Node):
    def __init__(self):
        super().__init__("action_manager_node")
        self._init_parameters()
        self._init_publishers()
        self._init_server_actions()
        self._init_services()
        self.get_logger().info("Server manager initialized.")

    def _init_parameters(self) -> None:
        self.declare_parameters(
            namespace="",
            parameters=[
                ("moveto_action", rclpy.Parameter.Type.STRING),
            ],
        )
        self.blocking = False

        if self.blocking:
            self._goal_queue = collections.deque()
            self._goal_queue_lock = threading.Lock()
            self._current_goal = None
            self.handle_accept_callback = self._queue_handle_accept_callback
        else:
            self._goal_lock = threading.Lock()
            self._goal_handle = None
            self.handle_accept_callback = self._single_handle_accept_callback

    def _init_publishers(self):
        self.pub_real_position = self.create_publisher(Point, "real_position_topic", 10)

    def _init_server_actions(self):
        self.get_logger().info("Receiving on moveto_action")
        self.moveto_server = ActionServer(
            self,
            ActionSequencer,
            self.get_namespace() + "/" + "moveto_action",
            self.execute_callback,
            handle_accept_callback=self.handle_accept_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup(),
        )

    def _init_services(self):
        self.srv_trigger_position = self.create_service(
            PosTrigger, "trigger_position_srv", self._send_position
        )

    def _send_position(self, request, response):
        response.pos.x = float(self.position[0])
        response.pos.y = float(self.position[1])
        response.pos.z = float(self.angle)
        return response

    def _single_handle_accepted_callback(self, goal_handle):
        self.get_logger().info("Im here")
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
        pass


def main(args=None):
    rclpy.init(args=args)
    action_manager_node = ActionManager()
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(action_manager_node, executor=executor)
    except Exception:
        pass
    finally:
        action_manager_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
