#!/usr/bin/env python3
"""Control the robot with the keyboard."""

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
import threading
from inputs import get_key
import time

class KeyboardController(Node):
    """Control robot with keyboard."""

    def __init__(self) -> None:
        super().__init__("keyboard_controller")
        self._init_parameters()
        self._init_publishers()
        self._init_subscribers()
        self.thread = threading.Thread(
                        target=self.wait_for_touch, # args=(self,)
                    )
        self.thread.start()
        self.get_logger().info("Controller keyboard for the robot initialized.")

    def _init_parameters(self) -> None:
        """Initialize parameters."""
        self.declare_parameters(
            namespace="",
            parameters=[
                ("robot_data_topic", "/main_robot/robot_data"),
                ("command_topic", "/main_robot/command"),
            ],
        )
        self.au = False
        self.speed = []
        self.control_speed = None
        self.robot_data = None

    def _init_publishers(self) -> None:
        """Initialize publishers."""
        self.command_topic = (
            self.get_parameter("command_topic").get_parameter_value().string_value
        )
        self.pub_command = self.create_publisher(String, self.command_topic, 10)

    def _init_subscribers(self) -> None:
        """Initialize subscribers."""
        self.robot_data_topic = (
            self.get_parameter("robot_data_topic").get_parameter_value().string_value
        )
        self.sub_robot_data = self.create_subscription(
            RobotData, self.robot_data_topic, self.robot_data_callback, 10
        )
        self.sub_au = self.create_subscription(
            Bool, "/main_robot/au", self.update_au, 10
        )

    # def wait_for_touch(self):
    #     # 2 controls mode with keyboard, on speed after clicking on 0: 
    #     # arrows: up, speed y + 0.05, 
    #     # down, speed y - 0.05, 
    #     # left, speed x - 0.05, 
    #     # right speed x + 0.05
    #     # touch o: speed rotation + 0.01 
    #     # touch p: speed rotation - 0.01

    #     # on Move after clicking on 1: 
    #     # arrows: up, pos y + 0.05, 
    #     # down, pos y - 0.05, 
    #     # left, pos x - 0.05, 
    #     # right pos x + 0.05
    #     # touch o: pos rotation + 0.01 
    #     # touch p: pos rotation - 0.01

    #     # If q pressed, send block and set to 0 speeds. 
    #     pass
    def print_help(self):
        print("\n=== KEYBOARD CONTROL HELP ===")
        print("Press C: SPEED CONTROL mode")
        print("Press V: MOVE TO POSITION mode")
        print("Key I:   +Y")
        print("Key K:   -Y")
        print("Key J:   -X")
        print("Key K:   +X")
        print("Key O:   +Rotation")
        print("Key P:   -Rotation")
        print("Key Q:   Send BLOCK command (emergency stop)")
        print("Key H:   Show this help message again\n")

    def wait_for_touch(self):
        self.print_help()
        mode = 0  # 0 = stopped, 1 = speed control, 2 = move control
        self.speed = [0.0, 0.0, 0.0]
        self.get_logger().info("Keyboard control started: press 0 for speed mode, 1 for move mode, q to block")
        while self.robot_data is None:
            time.sleep(0.2)
            self.get_logger().info("Waiting for robot data...")
        while rclpy.ok():
            if self.au:
                time.sleep(0.2)
            try:
                events = get_key()
                for event in events:
                    if event.ev_type != "Key" or event.state != 1:
                        continue
                    key = event.code
                    if key == "KEY_C":
                        mode = 1
                        self.get_logger().info("Switched to SPEED control mode")
                    elif key == "KEY_V":
                        mode = 2
                        self.get_logger().info("Switched to MOVE control mode")
                    elif key == "KEY_A":
                        self._send_block()
                        self.speed = [0.0, 0.0, 0.0]
                        self.get_logger().info("BLOCK command sent")
                    elif key == "KEY_H":
                        self.print_help()
                    elif key in ["KEY_I", "KEY_J", "KEY_K", "KEY_L", "KEY_O", "KEY_P"]:
                        if mode == 1:  # speed control
                            if key == "KEY_I":
                                self.speed[1] += 0.05
                            elif key == "KEY_K":
                                self.speed[1] -= 0.05
                            elif key == "KEY_J":
                                self.speed[0] -= 0.05
                            elif key == "KEY_K":
                                self.speed[0] += 0.05
                            elif key == "KEY_O":
                                self.speed[2] += 0.01
                            elif key == "KEY_P":
                                self.speed[2] -= 0.01
                            self._send_speed()
                            self.get_logger().info(f"SPEED: {self.speed}")
                        elif mode == 2:  # move control
                            pos = [0.0, 0.0, 0.0]
                            if key == "KEY_I":
                                pos[1] += 0.05
                            elif key == "KEY_K":
                                pos[1] -= 0.05
                            elif key == "KEY_J":
                                pos[0] -= 0.05
                            elif key == "KEY_K":
                                pos[0] += 0.05
                            elif key == "KEY_O":
                                pos[2] += 0.01
                            elif key == "KEY_P":
                                pos[2] -= 0.01
                            self._send_move(self.robot_data.x + pos[0], self.robot_data.y + pos[1], self.robot_data.theta + pos[2])
                            self.get_logger().info(f"MOVE TO: {self.robot_data.x + pos[0]} {self.robot_data.y + pos[1]} {self.robot_data.theta + pos[2]}")
            except Exception as e:
                self.get_logger().error(f"Keyboard input error: {e}")
                time.sleep(0.1)


    def update_au(self, msg):
        self.get_logger().info(f"I received a stop from AU")
        if msg.data:
            self.au = True
            self._send_block()
        else:
            self.au = False

    def robot_data_callback(self, msg: RobotData) -> None:
        """Retrieve the robot datas from Zynq."""
        self.robot_data = msg

    def _send_block(self) -> None:
        """Send block to motors."""
        if self.last_command_sent == "BLOCK":
            return
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

    def _send_speed(self) -> None:
        """Send move to motors."""
        cmd_msg = String()
        cmd_msg.data = f"ASPEED {self.speed[0]} {self.speed[1]} {self.speed[2]}"
        self.pub_command.publish(cmd_msg)
        
def main(args=None):
    """Spin main loop."""
    rclpy.init(args=args)
    node = KeyboardController()
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
