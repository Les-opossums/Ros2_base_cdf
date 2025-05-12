#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterEvent

from std_msgs.msg import String

from opossum_action_sequencer.utils import *


class ActionManager(Node):
    def __init__(self):
        super().__init__("action_sequencer_node")
        self.get_logger().info("Action Manager Node started")
        self._init_parameters()
        self._init_publishers()
        self._init_subscribers()
        self.state_leash = False
        self.script_class = None

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

    def _init_subscribers(self):
        self.subscription = self.create_subscription(
            ParameterEvent,
            '/parameter_events',
            self.parameter_event_callback,
            10)

        self.pub_feedback = self.create_subscription(String,
                                                     "/main_robot/feedback_command",
                                                     self.feedback_callback,
                                                     10
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
               #  Script.run(self)
                self.script_class = Script

    def feedback_callback(self, msg):
        # self.get_logger().info(f"Feedback received: {msg.data}")

        if msg.data.startswith("LEASH"):
            if self.ready:
                self.get_logger().info("Leash activated")
                # Script.run(self)
                self.script_class.run(self)
            self.state_leash = True

    def move_to(self, pos: Position):
        self.get_logger().info(f"Moving to : {pos.x} {pos.y} {pos.t}")
        self.pub_command.publish(String(
            data=f"MOVE {pos.x} {pos.y} {pos.t}"
        ))

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
        self.pub_command.publish(String(
            data=raw_command
        ))


def main(args=None):
    rclpy.init(args=args)
    action_manager_node = ActionManager()
    rclpy.spin(action_manager_node)
    action_manager_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
