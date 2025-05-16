#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterEvent

from std_msgs.msg import String, Bool, Int32
from geometry_msgs.msg import Point
from cdf_msgs.msg import LidarLoc

from opossum_action_sequencer.utils import *

import threading
from threading import Event


class ActionManager(Node):
    def __init__(self):
        super().__init__("action_sequencer_node")
        self.get_logger().info("Action Manager Node started")
        self._init_parameters()
        self._init_publishers()
        self._init_subscribers()
        self.state_leash = False
        self.script_class = None

        # Action Done
        self.is_robot_moving = False
        self.is_pump_top_on = False
        self.is_pump_bottom_on = False

        self.motion_done_event = Event()
        self.motion_done_event.set()

    def _init_parameters(self) -> None:
        self.declare_parameters(
            namespace="",
            parameters=[
                ("pub_command", "command"),
                ("feedback_topic", "pub_feedback_command"),
            ],
        )

    def _init_publishers(self):
        self.pub_command = self.create_publisher(String,
                                                 "/main_robot/command",
                                                 10
                                                 )

        self.pub_score = self.create_publisher(Int32,
                                               "/main_robot/score",
                                               10
                                               )

        self.pub_au = self.create_publisher(Bool,
                                            "/main_robot/au",
                                            10
                                            )

    def _init_subscribers(self):
        self.subscription = self.create_subscription(
            ParameterEvent,
            '/parameter_events',
            self.parameter_event_callback,
            10
        )

        self.pub_feedback = self.create_subscription(
            String,
            "/main_robot/feedback_command",
            self.feedback_callback,
            10
        )

        self.pos_sub = self.create_subscription(
            LidarLoc,
            "/main_robot/position_out",
            self.position_callback,
            10
        )

        self.velocity_sub = self.create_subscription(
            Point,
            "/main_robot/asserv/vel",
            self.velocity_callback,
            1
        )

    def parameter_event_callback(self, event):
        # Parcours des paramètres modifiés
        self.get_logger().info(f"Parameter event received: {event}")
        for changed in event.changed_parameters:
            if changed.name == 'script_number':
                # Affiche la nouvelle valeur du paramètre script_number
                self.get_logger().info(
                    f"Choix du script : {changed.value.integer_value}"
                )
                # Import script
                if changed.value.integer_value == 0:
                    pass
                elif changed.value.integer_value == 1:
                    from opossum_action_sequencer.match.script1 import Script
                    self.get_logger().info(f'Script 1')
                elif changed.value.integer_value == 2:
                    from opossum_action_sequencer.match.script2 import Script
                    self.get_logger().info(f'Script 2')
                else:
                    from opossum_action_sequencer.match.script1 import Script
                    self.get_logger().info(f'Default script')

                if changed.value.integer_value != 0:
                    self.ready = True
                    self.script_class = Script
                else:
                    pass

    def feedback_callback(self, msg):
        # self.get_logger().info(f"Feedback received: {msg.data}")

        if msg.data.startswith("LEASH"):
            # if self.ready:
            #     self.get_logger().info("Leash activated")
            #     # Script.run(self)
            #     self.script_class.run(self)
            self.state_leash = True
            if msg.data.startswith("LEASH"):
                if self.ready:
                    self.get_logger().info("Leash activated")
                    # thread = threading.Thread(target=self.script_class.run, 
                    #                           args=(self,)
                    #                           )
                    self.script_instance = self.script_class()
                    thread = threading.Thread(target=self.script_instance.run, 
                                              args=(self,))
                    thread.start()
                self.state_leash = True

        elif msg.data.startswith("AU"):
            self.get_logger().info(f"AU_test : {msg.data[-1]}")
            if msg.data[-1] == '1':
                self.get_logger().info("AU activated")
                self.pub_au.publish(Bool(data=True))
            else:
                self.get_logger().info("AU deactivated")
                self.pub_au.publish(Bool(data=False))

    def position_callback(self, msg: LidarLoc):
        # self.get_logger().info(f"Position received: {msg}")
        self.pos_lidar = Position(
            x=msg.robot_position.x,
            y=msg.robot_position.y,
            t=msg.robot_position.z,
        )

    def velocity_callback(self, msg: Point):
        # self.get_logger().info(f"Velocity received: x={msg.x}, y={msg.y}, z={msg.z}")
        if self.is_robot_moving:
            if abs(msg.x) < 0.0001 and abs(msg.y) < 0.0001 and abs(msg.z) < 0.0001:
                self.get_logger().info("Robot stopped")
                self.is_robot_moving = False
                self.motion_done_event.set()

    def move_to(self, pos: Position):
        self.get_logger().info(f"Moving to : {pos.x} {pos.y} {pos.t}")
        self.is_robot_moving = True
        self.pub_command.publish(String(
            data=f"MOVE {pos.x} {pos.y} {pos.t}"
        ))
        self.pos_obj = pos
        self.motion_done_event.clear()  # Block the wait
        self.get_logger().info(f"Robot moving...")

    def wait_for_motion(self):
        self.get_logger().info("Waiting for robot to stop...")
        self.motion_done_event.wait()
        self.get_logger().info("Motion done")

    def servo(self, servo: SERVO_struct):
        self.pub_command.publish(String(
            data=f"SERVO {servo.servo_id} {servo.angle}"
        ))

    def pump(self, pump: PUMP_struct):
        self.pub_command.publish(String(
            data=f"PUMP {pump.pump_id} {pump.enable}"
        ))

    def led(self, led: LED_struct):
        self.pub_command.publish(String(
            data=f"LED {led.red} {led.green} {led.blue}"
        ))

    def write_log(self, message):
        self.get_logger().info(f"{message}")

    def send_raw(self, raw_command):
        self.get_logger().info(f"Sending raw command: {raw_command}")
        self.pub_command.publish(String(
            data=raw_command
        ))

    def synchro_lidar(self):
        command = f"SYNCHROLIDAR {self.pos_lidar.x} {self.pos_lidar.y} {self.pos_lidar.t}"
        self.get_logger().info(f"Synchro : {command}")
        self.pub_command.publish(String(
            data=f"{command}"
        ))

    def add_score(self, score):
        self.pub_score.publish(Int32(
            data=score
        ))


def main(args=None):
    rclpy.init(args=args)
    action_manager_node = ActionManager()
    rclpy.spin(action_manager_node)
    action_manager_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
