#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Action Sequencer Node."""


import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterEvent

from std_msgs.msg import String, Bool, Int32
from opossum_msgs.msg import RobotData, LidarLoc

import numpy as np
from opossum_action_sequencer.utils import (
    Position,
    PUMP_struct,
    LED_struct,
    SERVO_struct,
    STEPPER_struct,
    VALVE_struct,
)

import multiprocessing
from multiprocessing import Event, Manager
import threading
from threading import Event
import time
import os


class ActionManager(Node):
    """Action Manager Node."""

    def __init__(self):
        super().__init__("action_sequencer_node")
        self.get_logger().info("Action Manager Node started")
        self._init_parameters()
        self._init_publishers()
        self._init_subscribers()
        self.state_leash = False
        self.script_class = None
        self.ready = False
        self.is_started = False
        self.match_time = None
        self.timer_move = None

        self.x_enn = None
        self.y_enn = None
        
        # Action Done
        self.is_robot_moving = False
        self.is_robot_arrived = False
        self.is_pump_top_on = False
        self.is_pump_bottom_on = False
        self.robot_pos = None
        self.motion_done = True

        # Process
        self.motion_done_event = Event()
        self.motion_done_event.set()

        self.end_match_event = Event()
        self.end_match_event.set()
        self.stop = False

    def _init_parameters(self) -> None:
        """Initialize the parameters of the node."""
        self.declare_parameters(
            namespace="",
            parameters=[
                ("pub_command", "command"),
                ("feedback_topic", "/main_robot/feedback_command"),
                ("pub_score", "/main_robot/score"),
                ("pub_au", "/main_robot/au"),
                ("pub_feedback", "/main_robot/feedback_command"),
                ("pub_position", "/main_robot/position_out"),
                ("pub_velocity", "/main_robot/asserv/vel"),
            ],
        )

    def _init_publishers(self):
        """Initialize the publishers of the node."""
        self.pub_command = self.create_publisher(
            String,
            "/main_robot/command",
            10
        )

        self.pub_score = self.create_publisher(
            Int32,
            "/main_robot/score",
            10
        )

        self.pub_au = self.create_publisher(
            Bool,
            "/main_robot/au",
            10
        )

        self.pub_end_of_match = self.create_publisher(
            Bool,
            "/main_robot/end_of_match",
            10
        )

    def _init_subscribers(self):
        """Initialize the subscribers of the node."""
        self.subscription = self.create_subscription(
            ParameterEvent,
            "/parameter_events",
            self.parameter_event_callback,
            10
        )

        self.sub_feedback = self.create_subscription(
            String,
            "/main_robot/feedback_command",
            self.feedback_callback,
            10
        )

        self.robot_data_sub = self.create_subscription(
            RobotData,
            "/main_robot/robot_data",
            self.robot_data_callback,
            10
        )

        self.lidar_loc_sub = self.create_subscription(
            LidarLoc,
            "/main_robot/position_out",
            self.lidar_loc_callback,
            10
        )

    def parameter_event_callback(self, event):
        """Handle the parameters event."""
        # Parcours des paramètres modifiés
        # self.get_logger().info(f"Parameter event received: {event}")
        for changed in event.changed_parameters:
            if changed.name == "script_number":
                # Affiche la nouvelle valeur du paramètre script_number
                self.get_logger().info(
                    f"Choix du script : {changed.value.integer_value}"
                )
                # Import script
                if changed.value.integer_value == 0:
                    pass
                elif changed.value.integer_value == 1:
                    from opossum_action_sequencer.match.script1 import Script

                    self.get_logger().info("Script 1")
                elif changed.value.integer_value == 2:
                    from opossum_action_sequencer.match.script2 import Script

                    self.get_logger().info("Script 2")

                elif changed.value.integer_value == 3:
                    from opossum_action_sequencer.match.script3 import Script

                    self.get_logger().info("Script 3")

                elif changed.value.integer_value == 4:
                    from opossum_action_sequencer.match.script4 import Script

                    self.get_logger().info("Script 4")

                elif changed.value.integer_value == 5:
                    from opossum_action_sequencer.match.script5 import Script

                    self.get_logger().info("Script 5")

                elif changed.value.integer_value == 6:
                    from opossum_action_sequencer.match.script6 import Script

                    self.get_logger().info("Script 6")

                elif changed.value.integer_value == 7:
                    from opossum_action_sequencer.match.script_homologation import Script

                    self.get_logger().info("Script Homologation")

                elif changed.value.integer_value == 9:
                    from opossum_action_sequencer.match.follow_ennemi import Script

                    self.get_logger().info("Script Follow Ennemi")

                else:
                    from opossum_action_sequencer.match.script1 import Script

                    self.get_logger().info("Default script")

                if changed.value.integer_value != 0:
                    self.ready = True
                    self.script_class = Script
                else:
                    pass

            elif changed.name == "debug_mode":
                # Affiche la nouvelle valeur du paramètre debug_mode
                if changed.value.bool_value:
                    self.get_logger().info("Debug mode enabled")
                    from opossum_action_sequencer.match.script_init import Script as ScriptInit
                    self.script_init_class = ScriptInit
                    self.run_script_init()
                else:
                    self.get_logger().info("Debug mode disabled")

    def feedback_callback(self, msg):
        """Receive the data from Zynq."""
        # self.get_logger().info(f"Feedback received: {msg.data}")

        if msg.data.startswith("LEASH"):
            self.state_leash = True
            if self.ready:
                self.is_started = True
                self.match_time = time.time()
                self.get_logger().info("Leash activated")
                self.script_instance = self.script_class()
                self.script_thread = threading.Thread(
                    target=self.script_instance.run, args=(self,)
                )
                self.script_thread.start()

        elif msg.data.startswith("AU"):
            # self.get_logger().info(f"AU_test : {msg.data[-1]}")
            if msg.data[-1] == "1":
                self.get_logger().info("AU activated")
                self.pub_au.publish(Bool(data=True))
                if self.is_started and self.script_thread is not None:
                    self.stop = True
                    self.get_logger().warn("Stopping script")
                    self.script_instance = None
                    self.script_thread = None
                    self.send_raw("BLOCK")
                    self.end_match_event.set()
                    time.sleep(0.1)
                    self.send_raw("BLOCK")
            elif msg.data[-1] == "2":
                # self.get_logger().info("AU activated")
                self.pub_au.publish(Bool(data=True))
            else:
                self.get_logger().info("AU deactivated")
                self.pub_au.publish(Bool(data=False))

        elif msg.data.startswith("YELLOWSWITCH"):
            self.synchro_lidar()

        elif msg.data.startswith("BLUESWITCH"):
            self.get_logger().info("Reload Ros")
            os.system('systemctl --user restart launch.service')

        elif msg.data.strip() == "Pos,done":
            self.get_logger().info("Motion done message received from Zynq")
            self.get_logger().info("Robot position from Zynq: "
                                   f"{self.robot_pos.x} {self.robot_pos.y} "
                                   f"{self.robot_pos.t}")
            self.motion_done = True
            self.motion_done_event.set()

    def robot_data_callback(self, msg: RobotData):
        """Receive the Robot Data from Zynq."""
        self.robot_pos = Position(x=msg.x, y=msg.y, t=msg.theta)
        self.robot_speed = Position(x=msg.vlin, y=msg.vdir, t=msg.vt)

        if not self.motion_done:
            # self.get_logger().info(f"Verifying motion status: "
            #                        f"{self.is_robot_moving} {self.motion_done}")
            # Update motion state
            self.update_arrival_status()
            self.update_motion_status()

            if not self.is_robot_moving and not self.motion_done:
                if self.timer_move is None:
                    self.timer_move = time.time()
                delta_time = time.time() - self.timer_move
                if delta_time > 2.0:
                    self.get_logger().warn(
                        "Motion timed out."
                    )
                    self.move_to(Position(self.pos_obj.x,
                                          self.pos_obj.y,
                                          self.pos_obj.t)
                                 )

        # Handle match time
        if self.is_started and self.match_time is not None:
            if self.script_thread is not None:
                self.current_time = time.time() - self.match_time
                if self.current_time > 95.0:
                    # End of match
                    self.pub_end_of_match.publish(Bool(data=True))
                    self.get_logger().warn("End of time, stopping script.")
                    self.send_raw("BLOCK")
                    self.end_match_event.set()
                    self.stop = True
                    time.sleep(0.1)
                    self.send_raw("BLOCK")
                    self.script_instance = None
                    self.script_thread = None

    def run_script_init(self):
        """Run the script initialization."""
        assert self.script_init_class is not None
        self.script_init_instance = self.script_init_class()
        self.process_init = threading.Thread(
            target=self.script_init_instance.run, args=(self,)
        )
        self.process_init.start()

    def lidar_loc_callback(self, msg: LidarLoc):
        """Receive Lidar location."""
        self.lidar_pos = Position(
            x=msg.robot_position.x,
            y=msg.robot_position.y,
            t=msg.robot_position.z
        )
        if len(msg.other_robot_position) == 0:
            self.x_enn = None
            self.y_enn = None
            return
        closer = np.sqrt((msg.other_robot_position[0].x - msg.robot_position.x) ** 2 + (msg.other_robot_position[0].y - msg.robot_position.y) ** 2)
        self.x_enn = msg.other_robot_position[0].x
        self.y_enn = msg.other_robot_position[0].y
        for pos in msg.other_robot_position:
            dst = np.sqrt((pos.x - msg.robot_position.x) ** 2 + (pos.y - msg.robot_position.y) ** 2)
            if dst < closer:
                self.x_enn = pos.x
                self.y_enn = pos.y

    def move_to(self, pos: Position, seuil=0.1, params=None):
        """Compute the move_to action."""
        if not self.stop:
            self.timer = None
            self.seuil = seuil
            self.get_logger().info(f"Moving to : {pos.x} {pos.y} {pos.t}")
            self.motion_done = False
            self.is_robot_moving = True
            self.is_robot_arrived = False
            time.sleep(0.1)
            if params is not None:
                command = f"MOVE {pos.x} {pos.y} {pos.t} {' '.join(params)}"
            else:
                command = f"MOVE {pos.x} {pos.y} {pos.t}"
            self.pub_command.publish(String(data=command))
            self.pos_obj = Position(x=pos.x, y=pos.y, t=pos.t)
            self.motion_done_event.clear()  # Block the wait

    def follow_ennemi(self):
        if not self.stop:
            x_middle = 1.5
            y_middle = 1.0
            while True:
                if self.shared_data["x_enn"] is not None:
                    self.send_raw("VMAX 0.7")
                    v1_x = x_middle - self.x_enn
                    v1_y = y_middle - self.y_enn
                    v1_norm = np.sqrt(v1_x ** 2 + v1_y ** 2)
                    v1_x /= v1_norm
                    v1_y /= v1_norm
                    pos_x = self.x_enn + v1_x * 0.5
                    pos_y = self.y_enn + v1_y * 0.5
                    dir = np.arctan2(v1_y, v1_x)
                    self.move_to(Position(pos_x, pos_y, dir))
                    self.get_logger().info(f"Move to ennemi: {pos_x} {pos_y} {dir}")
                    time.sleep(0.1)
                else:
                    self.send_raw("BLOCK")
                    self.get_logger().info(f"Sending BLOCK")
                    time.sleep(0.1)

                # self.get_logger().info("Robot moving...")

    def relative_move_to(self, delta: Position, seuil=0.1):
        """Compute the relative move_to action."""
        if not self.stop:
            self.timer = None
            self.seuil = seuil
            pos = Position(
                x=self.robot_pos.x + delta.x,
                y=self.robot_pos.y + delta.y,
                t=self.robot_pos.t + delta.t
            )
            self.get_logger().info(f"Robot_data : {self.robot_pos.x} {self.robot_pos.y} {self.robot_pos.t}")
            self.get_logger().info(f"Moving to : {pos.x} {pos.y} {pos.t}")
            self.motion_done = False
            self.is_robot_moving = True
            self.is_robot_arrived = False
            time.sleep(0.1)
            self.pub_command.publish(String(data=f"MOVE {pos.x} {pos.y} {pos.t}"))
            self.pos_obj = Position(x=pos.x, y=pos.y, t=pos.t)
            self.motion_done_event.clear()  # Block the wait
            # self.get_logger().info("Robot moving...")

    def update_arrival_status(self):
        if not self.stop:
            # self.get_logger().info("Updating arrival status...")
            time.sleep(0.2)
            delta_x = abs(self.pos_obj.x - self.robot_pos.x)
            delta_y = abs(self.pos_obj.y - self.robot_pos.y)
            delta_t = abs(self.pos_obj.t - self.robot_pos.t)
            if delta_x < 0.5 and delta_y < 0.5 and delta_t < 0.5:
                if not self.is_robot_arrived:
                    self.get_logger().info("Robot has arrived.")
                self.is_robot_arrived = True

    def update_motion_status(self):
        if not self.stop:
            # self.get_logger().info("Updating motion status...")
            time.sleep(0.2)
            # self.get_logger().info(f"Robot speed: vlin={self.robot_speed.x}, vt={self.robot_speed.t}")
            if self.robot_speed.x < 0.0001 and self.robot_speed.t < 0.0001:
                if self.is_robot_moving:
                    self.get_logger().info("Robot has stopped.")
                self.is_robot_moving = False

    def wait_for_motion(self):
        """Compute the wait_for_motion action."""
        if not self.stop:
            # self.get_logger().info("Waiting for robot to stop...")
            self.motion_done_event.wait()
            time.sleep(0.2)
            self.get_logger().info("Motion done from Ros")
            self.motion_done = True

    def servo(self, servo: SERVO_struct):
        """Compute the servo action."""
        if not self.stop:
            self.pub_command.publish(String(data=f"SERVO {servo.servo_id} "
                                                 f"{servo.angle}"
                                            )
                                     )

            self.get_logger().info(f"SERVO : SERVO {servo.servo_id} {servo.angle}")
            time.sleep(0.1)

    def pump(self, pump: PUMP_struct):
        """Compute the pump action."""
        if not self.stop:
            self.pub_command.publish(String(data=f"PUMP {pump.pump_id} "
                                                 f"{pump.enable}"
                                            )
                                     )

            self.get_logger().info(f"PUMP : PUMP {pump.pump_id} {pump.enable}")
            time.sleep(0.1)

    def led(self, led: LED_struct):
        """Compute the led action."""
        self.pub_command.publish(String(data=f"LED {led.red} "
                                             f"{led.green} {led.blue}")
                                 )

    def stepper(self, stepper: STEPPER_struct):
        """Compute the stepper action."""
        if not self.stop:
            self.pub_command.publish(
                String(data=f"STEPPER1  {stepper.mode}")
            )
            self.get_logger().info(f"STEPPER : STEPPER1 {stepper.mode}")
            time.sleep(0.1)

    def valve(self, valve: VALVE_struct):
        """Compute the valve action."""
        if not self.stop:
            self.pub_command.publish(String(data=f"VALVE {valve.valve_id} 1"))

            self.get_logger().info(f"VALVE {valve.valve_id} 1")
            time.sleep(0.1)

    def write_log(self, message):
        """Write logs."""
        if not self.stop:
            self.get_logger().info(f"{message}")

    def sleep(self, duration):
        """Sleep for a given duration."""
        if not self.stop:
            # self.get_logger().info(f"Sleeping for {duration} seconds")
            time.sleep(duration)

    def send_raw(self, raw_command):
        """Send raw commands."""
        if not self.stop:
            self.get_logger().info(f"Sending raw command: {raw_command}")
            self.pub_command.publish(String(data=raw_command))
            time.sleep(0.1)

    def kalman(self, kalman: bool):
        """Compute the kalman action."""
        if not self.stop:
            if kalman:
                self.pub_command.publish(String(data="ENKALMAN 1"))
                self.get_logger().info("KALMAN ON")
            else:
                self.pub_command.publish(String(data="ENKALMAN 0"))
                self.get_logger().info("KALMAN OFF")
            time.sleep(0.1)

    def synchro_lidar(self):
        """Synchronize odom with lidar."""
        if not self.stop:
            if self.robot_pos is None:
                return
            command = (
                f"SYNCHROLIDAR {self.lidar_pos.x} "
                f"{self.lidar_pos.y} {self.lidar_pos.t}"
            )
            self.get_logger().info(f"Synchro : {command}")
            self.pub_command.publish(String(data=f"{command}"))

    def add_score(self, score):
        """Update the score."""
        if not self.stop:
            self.pub_score.publish(Int32(data=score))
            time.sleep(0.1)

    def smart_moves(self):
        pass
    
def main(args=None):
    """Run main loop."""
    rclpy.init(args=args)
    action_manager_node = ActionManager()
    rclpy.spin(action_manager_node)
    action_manager_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
