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
        self.script_class = None
        self.ready = False
        self.is_started = False
        self.is_ended = False
        self.match_time = None
        self.timer_move = None
        self.in_end_zone = False
        self.end_zone = None
        self.middle_zone = None

        self.x_enn = None
        self.y_enn = None

        # Action Done
        self.is_robot_moving = False
        self.is_robot_arrived = False
        self.is_pump_top_on = False
        self.is_pump_bottom_on = False
        self.robot_pos = None
        self.motion_done = True
        self.color = None

        # Process
        self.motion_done_event = Event()
        self.motion_done_event.set()

        self.end_match_event = Event()
        self.end_match_event.set()
        self.stop = False

        self.end_poses = {
            0: [0, 0.875, 2],
            1: [0.225, 0, 1],
            2: [1.775, 0, 1],
            3: [2.225, 0, 1],
        }
        self.end_poses = self.end_poses | {key + 4: [3 - val[0], val [1], 2 - val[2]] for key, val in self.end_poses.items()}
        self.available_end = {i: 0 for i in range(len(list(self.end_poses.keys())))}

        self.position_cans = {
            0: [0.075, 1.325, 2],
            1: [0.075, 0.4, 2],
            2: [0.825, 1.725, 1],
            3: [1.1, 0.95, 1],
            4: [0.775, 0.25, 1],
        }
        self.position_cans = self.position_cans | {key + 5: [3 - val[0], val[1], 2 - val[2]] for key, val in self.position_cans.items()}
        self.available_cans = {i: True for i in range(len(list(self.position_cans.keys())))}

        self.dest_cans = {
            0: [6, 0],
            1: [6, 0],
            2: [6, 0],
            3: [6, 0],
            4: [7, 1],
            5: [4, 2],
            6: [4, 2],
            7: [4, 2],
            8: [4, 2],
            9: [5, 3],
        }

    def _init_parameters(self) -> None:
        """Initialize the parameters of the node."""
        self.declare_parameters(
            namespace="",
            parameters=[
                ("command_topic", "command"),
                ("score_topic", "score"),
                ("au_topic", "au"),
                ("end_of_match_topic", "end_of_match"),
                ("feedback_command_topic", "feedback_command"),
                ("robot_data_topic", "robot_data"),
                ("position_topic", "position_out"),
                ("color_topic", "init_team_color"),
            ],
        )

    def _init_timers(self):
        """Initialize the timers of the node."""
        self.timer_match = self.create_timer(
            96,
            self.timer_match_callback
        )
        self.timer_backstage = self.create_timer(
            92,
            self.timer_backstage_callback
        )

    def _init_move_timer(self):
        self.move_timer = self.create_timer(
            2,
            self.timer_move_callback
        )

    def _reset_move_timer(self):
        """Reset the move timer."""
        if self.move_timer is not None:
            self.move_timer.cancel()
            self.move_timer = None

    def _init_publishers(self):
        """Initialize the publishers of the node."""
        command_topic = (
            self.get_parameter("command_topic")
            .get_parameter_value()
            .string_value
        )

        score_topic = (
            self.get_parameter("score_topic")
            .get_parameter_value()
            .string_value
        )

        au_topic = (
            self.get_parameter("au_topic")
            .get_parameter_value()
            .string_value
        )

        end_of_match_topic = (
            self.get_parameter("end_of_match_topic")
            .get_parameter_value()
            .string_value
        )

        self.pub_command = self.create_publisher(
            String,
            command_topic,
            10
        )

        self.pub_score = self.create_publisher(
            Int32,
            score_topic,
            10
        )

        self.pub_au = self.create_publisher(
            Bool,
            au_topic,
            10
        )

        self.pub_end_of_match = self.create_publisher(
            Bool,
            end_of_match_topic,
            10
        )

    def _init_subscribers(self):
        """Initialize the subscribers of the node."""

        feedback_command_topic = (
            self.get_parameter("feedback_command_topic")
            .get_parameter_value()
            .string_value
        )

        robot_data_topic = (
            self.get_parameter("robot_data_topic")
            .get_parameter_value()
            .string_value
        )

        position_topic = (
            self.get_parameter("position_topic")
            .get_parameter_value()
            .string_value
        )

        color_topic = (
            self.get_parameter("color_topic")
            .get_parameter_value()
            .string_value
        )

        self.subscription = self.create_subscription(
            ParameterEvent,
            "/parameter_events",
            self.parameter_event_callback,
            10
        )

        self.sub_feedback = self.create_subscription(
            String,
            feedback_command_topic,
            self.feedback_callback,
            10
        )

        self.robot_data_sub = self.create_subscription(
            RobotData,
            robot_data_topic,
            self.robot_data_callback,
            10
        )

        self.lidar_loc_sub = self.create_subscription(
            LidarLoc,
            position_topic,
            self.lidar_loc_callback,
            10
        )

        self.color_sub = self.create_subscription(
            String,
            color_topic,
            self.color_callback,
            10
        )

    def parameter_event_callback(self, event):
        """Handle the parameters event."""
        # Parcours des paramètres modifiés
        # self.get_logger().info(f"Parameter event received: {event}")
        script_map = {
            0: ("opossum_action_sequencer.match.script1",
                "Default Script"),
            1: ("opossum_action_sequencer.match.script1",
                "Script 1"),
            2: ("opossum_action_sequencer.match.script2",
                "Script 2"),
            3: ("opossum_action_sequencer.match.script3",
                "Script 3"),
            4: ("opossum_action_sequencer.match.script4",
                "Script 4"),
            5: ("opossum_action_sequencer.match.script5",
                "Script 5"),
            6: ("opossum_action_sequencer.match.script6",
                "Script 6"),
            7: ("opossum_action_sequencer.match.script_homologation",
                "Script Homologation"),
            9: ("opossum_action_sequencer.match.follow_ennemi",
                "Script Follow Ennemi"),
            11: ("opossum_action_sequencer.match.smart_script",
                 "Script Smart"),
            12: ("opossum_action_sequencer.match.smart_script",
                 "Script Smart"),
            69: ("opossum_action_sequencer.match.script_init",
                 "Script Init"),
        }

        for changed in event.changed_parameters:
            if changed.name == "script_number":
                script_num = changed.value.integer_value
                self.get_logger().info(f"Choix du script : {script_num}")

                if script_num in script_map:
                    module_path, log_msg = script_map[script_num]
                    self.get_logger().info(log_msg)

                    # Dynamically import the Script class
                    module = __import__(module_path, fromlist=["Script"])
                    Script = getattr(module, "Script")

                else:
                    self.get_logger().error("Aucun script associé à la valeur "
                                            f": {script_num}")
                    raise ValueError("Invalid script number")

                if changed.value.integer_value != 0:
                    self.ready = True
                    self.script_class = Script
                else:
                    pass

            elif changed.name == "debug_mode":
                # Affiche la nouvelle valeur du paramètre debug_mode
                if changed.value.bool_value:
                    self.get_logger().info("Debug mode enabled")
                    module = __import__(
                        "opossum_action_sequencer.match.script_init",
                        fromlist=["Script"]
                    )
                    self.script_init_class = getattr(module, "Script")
                    self.run_script_init()
                else:
                    self.get_logger().info("Debug mode disabled")

    def color_callback(self, msg):
        self.color = msg.data

    def feedback_callback(self, msg):
        """Receive the data from Zynq."""
        # self.get_logger().info(f"Feedback received: {msg.data}")
        if msg.data.startswith("LEASH") and self.ready and not self.is_started:
            self.get_logger().info(f"I REACEIVED LEASH FOR {self.get_namespace()}")
            self._init_timers()
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
                    self.get_logger().warn("Stop script from AU")
                    self.stop_script()
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
            # self.get_logger().info("Motion done message received from Zynq")
            # self.get_logger().info("Robot position from Zynq: "
            #                        f"{self.robot_pos.x} {self.robot_pos.y} "
            #                        f"{self.robot_pos.t}")
            self.motion_done = True
            self.motion_done_event.set()

    def robot_data_callback(self, msg: RobotData):
        """Receive the Robot Data from Zynq."""
        self.robot_pos = Position(x=msg.x, y=msg.y, t=msg.theta)
        self.robot_speed = Position(x=msg.vlin, y=msg.vdir, t=msg.vt)

        if False: #not self.motion_done:
            # self.get_logger().info(f"Verifying motion status: "
            #                        f"{self.is_robot_moving} {self.motion_done}")
            # Update motion state
            self.update_arrival_status()
            self.update_motion_status()
            self.get_logger().info(f"Robot is moving: {self.is_robot_moving}")
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
                    self.timer_move = None

    def timer_match_callback(self):
        """Timer callback for match time."""
        if not self.is_ended:
            self.pub_end_of_match.publish(Bool(data=True))
            self.get_logger().warn("Match time exceeded")
            self.stop_script()

    def timer_backstage_callback(self):
        if self.end_zone is not None and self.middle_zone is not None:
            if not self.is_ended and not self.in_end_zone:
                self.get_logger().warn("Abort, go back home")
                self.send_raw("VMAX 0.5")
                self.move_to(self.middle_zone)
                self.wait_for_motion()
                time.sleep(0.1)
                self.move_to(self.end_zone)

    def timer_move_callback(self):
        if not self.is_robot_moving and not self.motion_done:
            self._reset_move_timer()
            self.get_logger().warn("New Motion timed out.")
            self.move_to(self.pos_obj)
            self._init_move_timer()

    def run_script_init(self):
        """Run the script initialization."""
        assert self.script_init_class is not None
        self.script_init_instance = self.script_init_class()
        self.process_init = threading.Thread(
            target=self.script_init_instance.run, args=(self,)
        )
        self.process_init.start()

    def stop_script(self):
        """Stop the current script."""
        if self.script_instance is not None:
            self.get_logger().warn("Stopping script")
            self.stop = True
            self.is_ended = True
            self.send_raw("BLOCK")
            self.end_match_event.set()
            time.sleep(0.1)
            self.send_raw("BLOCK")
            self.script_instance = None
            self.script_thread = None

    def lidar_loc_callback(self, msg: LidarLoc):
        """Receive Lidar location."""
        self.lidar_pos = Position(
            x=msg.robot_position.x,
            y=msg.robot_position.y,
            t=msg.robot_position.z
        )
        list_enn = []
        for rob in msg.other_robot_position:
            if rob.x > 0.3 and rob.x < 2.7 and rob.y > 0.3 and rob.y < 1.7:
                list_enn.append(rob)
        if len(list_enn) == 0:
            self.x_enn = None
            self.y_enn = None
            return
        closer = np.sqrt((list_enn[0].x - msg.robot_position.x) ** 2 + (list_enn[0].y - msg.robot_position.y) ** 2)
        self.x_enn = list_enn[0].x
        self.y_enn = list_enn[0].y
        for pos in list_enn:
            dst = np.sqrt((pos.x - msg.robot_position.x) ** 2 + (pos.y - msg.robot_position.y) ** 2)
            if dst < closer:
                self.x_enn = pos.x
                self.y_enn = pos.y

    def move_to(self, pos: Position, seuil=0.1, params=None):
        """Compute the move_to action."""
        if not self.stop:
            self.timer = None
            self.seuil = seuil
            # self.get_logger().info(f"Moving to : {pos.x} {pos.y} {pos.t}")
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
                if self.x_enn is not None:
                    self.send_raw("VMAX 0.8")
                    v1_x = x_middle - self.x_enn
                    v1_y = y_middle - self.y_enn
                    v1_norm = np.sqrt(v1_x ** 2 + v1_y ** 2)
                    v1_x /= v1_norm
                    v1_y /= v1_norm
                    pos_x = self.x_enn # + v1_x * 0.5
                    pos_y = self.y_enn # + v1_y * 0.5
                    dir = np.arctan2(v1_y, v1_x)
                    self.move_to(Position(pos_x, pos_y, dir))
                    # self.get_logger().info(f"Move to ennemi: {pos_x} {pos_y} {dir}")
                    time.sleep(0.1)
                else:
                    self.send_raw("BLOCK")
                    # self.get_logger().info(f"Sending BLOCK")
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
            # self.get_logger().info(f"Robot_data : {self.robot_pos.x} {self.robot_pos.y} {self.robot_pos.t}")
            # self.get_logger().info(f"Moving to : {pos.x} {pos.y} {pos.t}")
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
            time.sleep(0.1)
            # self.get_logger().info("Updating arrival status...")
            delta_x = abs(self.pos_obj.x - self.robot_pos.x)
            delta_y = abs(self.pos_obj.y - self.robot_pos.y)
            delta_t = abs(self.pos_obj.t - self.robot_pos.t)
            if delta_x < 0.5 and delta_y < 0.5 and delta_t < 0.5:
                # if not self.is_robot_arrived:
                    # self.get_logger().info("Robot has arrived.")
                self.is_robot_arrived = True

    def update_motion_status(self):
        if not self.stop:
            time.sleep(0.1)
            if not self.is_robot_moving and not self.motion_done:
                # self.get_logger().info("Updating motion status...")
                if abs(self.robot_speed.x) < 0.0001 and abs(self.robot_speed.t) < 0.0001:
                    self.get_logger().info("Robot has stopped.")
                    self.get_logger().info(f"Robot speed: vlin={self.robot_speed.x}, vt={self.robot_speed.t}")
                    self.is_robot_moving = False
                    self._init_move_timer()

    def wait_for_motion(self):
        """Compute the wait_for_motion action."""
        if not self.stop:
            # self.get_logger().info("Waiting for robot to stop...")
            self.motion_done_event.wait()
            time.sleep(0.2)
            # self.get_logger().info("Motion done from Ros")
            self.motion_done = True

    def servo(self, servo: SERVO_struct):
        """Compute the servo action."""
        if not self.stop:
            self.pub_command.publish(String(data=f"SERVO {servo.servo_id} "
                                                 f"{servo.angle}"
                                            )
                                     )

            # self.get_logger().info(f"SERVO : SERVO {servo.servo_id} {servo.angle}")
            time.sleep(0.1)

    def pump(self, pump: PUMP_struct):
        """Compute the pump action."""
        if not self.stop:
            self.pub_command.publish(String(data=f"PUMP {pump.pump_id} "
                                                 f"{pump.enable}"
                                            )
                                     )

            # self.get_logger().info(f"PUMP : PUMP {pump.pump_id} {pump.enable}")
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
            # self.get_logger().info(f"STEPPER : STEPPER1 {stepper.mode}")
            time.sleep(0.1)

    def valve(self, valve: VALVE_struct):
        """Compute the valve action."""
        if not self.stop:
            self.pub_command.publish(String(data=f"VALVE {valve.valve_id} 1"))

            # self.get_logger().info(f"VALVE {valve.valve_id} 1")
            time.sleep(0.1)

    def write_log(self, message):
        """Write logs."""
        if not self.stop:
            # self.get_logger().info(f"{message}")
            pass

    def sleep(self, duration):
        """Sleep for a given duration."""
        if not self.stop:
            # self.get_logger().info(f"Sleeping for {duration} seconds")
            time.sleep(duration)

    def send_raw(self, raw_command):
        """Send raw commands."""
        if not self.stop:
            # self.get_logger().info(f"Sending raw command: {raw_command}")
            self.pub_command.publish(String(data=raw_command))
            time.sleep(0.1)

    def kalman(self, kalman: bool):
        """Compute the kalman action."""
        if not self.stop:
            if kalman:
                self.pub_command.publish(String(data="ENKALMAN 1"))
                # self.get_logger().info("KALMAN ON")
            else:
                self.pub_command.publish(String(data="ENKALMAN 0"))
                # self.get_logger().info("KALMAN OFF")
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
            # self.get_logger().info(f"Synchro : {command}")
            self.pub_command.publish(String(data=f"{command}"))

    def add_score(self, score):
        """Update the score."""
        if not self.stop:
            self.pub_score.publish(Int32(data=score))
            time.sleep(0.1)

    def sign(self, val):
        return -1 if val < 0 else 1

    def sign(self, val):
        return -1 if val < 0 else 1

    def smart_moves(self):
        default_angle = -2.60
        tol = 0.3
        vmax = 0.3
        if self.color.lower() == "yellow":
            en_color = 0
        elif self.color.lower() == "blue":
            en_color = 1
        else:
            self.get_logger().error(f"NO COLOR")
        while True:
            # self.get_logger().info("Sending new goal.")
            self.send_raw(f"VMAX {vmax}")
            max = -10
            can_valid = None
            ind_valid = None
            for ind, cans in self.position_cans.items():
                if self.available_cans[ind]:
                    temp = self.compute_penality(cans)
                    # self.get_logger().info(f"Reward for cans: {ind}, {temp}")
                    if temp > max:
                        max = temp
                        ind_valid = ind
                        can_valid = cans
            if can_valid is not None:
                if can_valid[2] % 2 == 0:
                    # HERE GO IN FRONT OF CANS THAT ARE BORDERLINE
                    self.move_to(Position(can_valid[0] + (can_valid[2] - 1) * tol, can_valid[1], can_valid[2] * np.pi / 2 + default_angle))
                    self.wait_for_motion()
                    fpos = self.find_final_pos(ind_valid, en_color)
                    self.get_logger().info(f"Take can id {ind_valid} at position {can_valid} with final position {fpos}")
                    self.take_cans(can_valid[2])
                    # self.get_logger().info(f"Drop")
                    self.drop_cans(fpos, default_angle)
                else: # CANS THAT ARE FRONT ON BOARD
                    if self.robot_pos.y > can_valid[1] or can_valid[1] < 0.5: # CHECK IF ROBOT ABOVE THE CANS OR CANS CLOSE TO BOUNDARIES
                        if self.robot_pos.y - can_valid[1] < tol:
                            self.move_to(Position(can_valid[0] + self.sign(self.robot_pos.x - can_valid[0]) * tol, can_valid[1] + tol, 3 * np.pi / 2 + default_angle))
                            self.wait_for_motion()
                        self.move_to(Position(can_valid[0], can_valid[1] + tol, 3 * np.pi / 2 + default_angle))
                        self.wait_for_motion()
                        fpos = self.find_final_pos(ind_valid, en_color)
                        self.get_logger().info(f"Take can id {ind_valid} at position {can_valid} with final position {fpos}")
                        self.take_cans(3)
                        # self.get_logger().info(f"Drop")
                        self.drop_cans(fpos, default_angle)
                    else:
                        if can_valid[1] - self.robot_pos.y < tol:
                            self.move_to(Position(can_valid[0] + self.sign(self.robot_pos.x - can_valid[0]) * tol, can_valid[1] - tol, np.pi / 2 + default_angle))
                            self.wait_for_motion()
                        self.move_to(Position(can_valid[0], can_valid[1] - tol, np.pi / 2 + default_angle))
                        self.wait_for_motion()
                        fpos = self.find_final_pos(ind_valid, en_color)
                        self.get_logger().info(f"Take can id {ind_valid} at position {can_valid} with final position {fpos}")
                        self.take_cans(1)
                        # self.get_logger().info(f"Drop")
                        self.drop_cans(fpos, default_angle)
                self.available_cans[ind_valid] = False
                self.available_end[self.dest_cans[ind_valid][en_color]] += 1 # If yellow 0, else blue
                # self.get_logger().info(f"Available cans now: {self.available_cans}")
            else:
                if en_color == 0: # color is yellow
                    self.move_to(Position(0.5, 1.4, 0))
                    self.wait_for_motion()
                    self.move_to(Position(0.5, 1.7, 0))
                    self.wait_for_motion()
                    self.send_raw("BLOCK")
                    break
                else:
                    self.move_to(Position(3 - 0.5, 1.4, 0))
                    self.wait_for_motion()
                    self.move_to(Position(3 - 0.5, 1.7, 0))
                    self.wait_for_motion()
                    self.send_raw("BLOCK")

    def find_final_pos(self, index, en_color):
        size_cans = 0.1
        size_robot = 0.27
        pos_out = self.end_poses[self.dest_cans[index][en_color]] # If yellow 0
        incr = self.available_end[self.dest_cans[index][en_color]]
        if pos_out[2] % 2 == 0:
            return [pos_out[0] + (pos_out[2] - 1) * (size_cans / 2 + size_robot + size_cans * incr), pos_out[1], pos_out[2]]
        else:
            return [pos_out[0], pos_out[1] + size_cans / 2 + size_robot + size_cans * incr, 3 * pos_out[2]]

    def angular_distance(a1, a2):
        diff = (a2 - a1 + np.pi) % (2 * np.pi) - np.pi
        return abs(diff)

    def compute_penality(self, pos):
        angle_coeff = 0.05
        coeff_center = 0 # -0.005
        coeff_dst = -1
        coeff_enn = 0 # 0.05
        # coeef_end = -0.0001
        val_center = (1.5 - pos[0]) ** 2 + (1 - pos[1]) ** 2
        if pos[2] % 2 == 0:
            angle = pos[2] * np.pi / 2
        elif self.robot_pos.y > pos[1] and pos[1] > 0.5:
            angle = 3 * np.pi / 2
        else:
            angle = np.pi / 2
        val_dst = (self.robot_pos.x - pos[0]) ** 2 + (self.robot_pos.y - pos[1]) ** 2 + angle_coeff * (self.robot_pos.t - angle) ** 2
        if self.x_enn is not None:
            val_ennemi = (self.x_enn - pos[0]) ** 2 + (self.y_enn - pos[1]) ** 2
        else:
            val_ennemi = 0
        return coeff_dst * val_dst + coeff_enn * val_ennemi + coeff_center * val_center

    def take_cans(self, angle):
        self.kalman(False)
        self.pump(PUMP_struct(1, 1))
        self.sleep(0.1)
        push_dst = 0.1
        if angle == 0:
            self.relative_move_to(Position(push_dst, 0, 0))
        elif angle == 1:
            self.relative_move_to(Position(0, push_dst, 0))
        elif angle == 2:
            self.relative_move_to(Position(-push_dst, 0, 0))
        elif angle == 3:
            self.relative_move_to(Position(0, -push_dst, 0))
        else:
            self.relative_move_to(Position(0, 0, 0))
            self.get_logger().error(f"Invalid angle for taking cans: {angle}")
            pass
        self.wait_for_motion()
        self.kalman(True)
        self.sleep(0.1)

    def drop_cans(self, destination, default_angle):
        push_dst = 0.1
        self.move_to(Position(destination[0], destination[1], destination[2] * np.pi / 2 + default_angle))
        self.wait_for_motion()

        self.pump(PUMP_struct(1, 0))
        self.valve(VALVE_struct(2))
        if destination[2] == 0:
            self.relative_move_to(Position(push_dst, 0, 0))
            self.wait_for_motion()
            self.relative_move_to(Position(-push_dst, 0, 0))
            self.wait_for_motion()
        elif destination[2] == 2:
            self.relative_move_to(Position(-push_dst, 0, 0))
            self.wait_for_motion()
            self.relative_move_to(Position(push_dst, 0, 0))
            self.wait_for_motion()
        elif destination[2] == 3:
            self.relative_move_to(Position(0, -push_dst, 0))
            self.wait_for_motion()
            self.relative_move_to(Position(0, push_dst, 0))
            self.wait_for_motion()
        else:
            self.get_logger().error(f"Invalid angle for dropping cans: "
                                    f"{destination[2]}")


def main(args=None):
    """Run main loop."""
    rclpy.init(args=args)
    action_manager_node = ActionManager()
    rclpy.spin(action_manager_node)
    action_manager_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
