#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Action Sequencer Node."""


import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterEvent

from std_msgs.msg import String, Bool, Int32
from geometry_msgs.msg import Point
from opossum_msgs.msg import RobotData, LidarLoc, VisionData

import numpy as np
from opossum_action_sequencer.utils import (
    Position,
    PUMP_struct,
    VACCUMGRIPPER_struct,
    LED_struct,
    SERVO_struct,
    STEPPER_struct,
    VALVE_struct,
)

import threading
from threading import Event
import time
import os
import json

class ActionManager(Node):
    """Action Manager Node."""

    def __init__(self):
        super().__init__("action_sequencer_node")
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

        self.x_tag = None
        self.y_tag = None

        self.new_pos = 0

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
            0: [0.3, 1.8, 2],
        }
        self.end_poses = self.end_poses | {key + 1: [3 - val[0], val [1], 2 - val[2]] for key, val in self.end_poses.items()}
        self.available_end = {i: 0 for i in range(len(list(self.end_poses.keys())))}

        # ID_object: ID_stack, X, Y, Theta, Time
        self.position_crates = {
            0: [0.1, 0.325, 2],
            1: [0.1, 0.375, 2],
            2: [0.1, 0.425, 2],
            3: [0.1, 0.475, 2],
            4: [0.1, 1.125, 2],
            5: [0.1, 1.175, 2],
            6: [0.1, 1.225, 2],
            7: [0.1, 1.275, 2],
            8: [1.025, 0.1, 1],
            9: [1.075, 0.1, 1],
            10: [1.125, 0.1, 1],
            11: [1.175, 0.1, 1],
            12: [1.825, 0.1, 1],
            13: [1.875, 0.1, 1],
            14: [1.925, 0.1, 1],
            15: [1.975, 0.1, 1],
            16: [1.075, 0.8, 1],
            17: [1.125, 0.8, 1],
            18: [1.175, 0.8, 1],
            19: [1.225, 0.8, 1],
            20: [1.775, 0.8, 1],
            21: [1.825, 0.8, 1],
            22: [1.875, 0.8, 1],
            23: [1.925, 0.8, 1],
            24: [2.9, 0.325, 0],
            25: [2.9, 0.375, 0],
            26: [2.9, 0.425, 0],
            27: [2.9, 0.475, 0],
            28: [2.9, 1.125, 0],
            29: [2.9, 1.175, 0],
            30: [2.9, 1.225, 0],
            31: [2.9, 1.275, 0],
        }

        # self.position_crates = {
        #     0: [0, 0.1, 0.325, 1.57, 0.],
        #     1: [0, 0.1, 0.375, 1.57, 0.],
        #     2: [0, 0.1, 0.425, 1.57, 0.],
        #     3: [0, 0.1, 0.475, 1.57, 0.],
        #     4: [1, 0.1, 1.125, 1.57, 0.],
        #     5: [1, 0.1, 1.175, 1.57, 0.],
        #     6: [1, 0.1, 1.225, 1.57, 0.],
        #     7: [1, 0.1, 1.275, 1.57, 0.],
        #     8: [2, 1.025, 0.1, 0.0, 0.],
        #     9: [2, 1.075, 0.1, 0.0, 0.],
        #     10: [2, 1.125, 0.1, 0.0, 0.],
        #     11: [2, 1.175, 0.1, 0.0, 0.],
        #     12: [3, 1.825, 0.1, 0.0, 0.],
        #     13: [3, 1.875, 0.1, 0.0, 0.],
        #     14: [3, 1.925, 0.1, 0.0, 0.],
        #     15: [3, 1.975, 0.1, 0.0, 0.],
        #     16: [4, 1.075, 0.8, 0.0, 0.],
        #     17: [4, 1.125, 0.8, 0.0, 0.],
        #     18: [4, 1.175, 0.8, 0.0, 0.],
        #     19: [4, 1.225, 0.8, 0.0, 0.],
        #     20: [5, 1.775, 0.8, 0.0, 0.],
        #     21: [5, 1.825, 0.8, 0.0, 0.],
        #     22: [5, 1.875, 0.8, 0.0, 0.],
        #     23: [5, 1.925, 0.8, 0.0, 0.],
        #     24: [6, 2.9, 0.325, 1.57, 0.],
        #     25: [6, 2.9, 0.375, 1.57, 0.],
        #     26: [6, 2.9, 0.425, 1.57, 0.],
        #     27: [6, 2.9, 0.475, 1.57, 0.],
        #     28: [7, 2.9, 1.125, 1.57, 0.],
        #     29: [7, 2.9, 1.175, 1.57, 0.],
        #     30: [7, 2.9, 1.225, 1.57, 0.],
        #     31: [7, 2.9, 1.275, 1.57, 0.],
        # }
        self.available_crates = {i: True for i in range(len(list(self.position_crates.keys())))}
        self.dest_crates = {i : [0, 1] for i in range(len(self.position_crates.keys()))}

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
                ("robot_data_topic", "/main_robot/robot_data"),
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

        self.aruco_sub = self.create_subscription(
            VisionData,
            "aruco_loc",
            self.aruco_callback,
            10
        )

        self.robot_data_sub = self.create_subscription(
            RobotData,
            robot_data_topic,
            self.robot_data_callback,
            10
        )

    def parameter_event_callback(self, event):
        """Handle the parameters event."""
        # Parcours des paramètres modifiés
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

                if script_num in script_map:
                    module_path, log_msg = script_map[script_num]
                    # self.get_logger().info(log_msg)

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
                    module = __import__(
                        "opossum_action_sequencer.match.script_init",
                        fromlist=["Script"]
                    )
                    self.script_init_class = getattr(module, "Script")
                    self.run_script_init()

    def color_callback(self, msg):
        self.color = msg.data

    def feedback_callback(self, msg):
        """Receive the data from Zynq."""
        if msg.data.startswith("LEASH") and self.ready and not self.is_started:
            self.get_logger().info(f"I REACEIVED LEASH FOR {self.get_namespace()}")
            self._init_timers()
            self.is_started = True
            self.match_time = time.time()
            self.script_instance = self.script_class()
            self.script_thread = threading.Thread(
                target=self.script_instance.run, args=(self,)
            )
            self.script_thread.start()

        elif msg.data.startswith("AU"):
            if msg.data[-1] == "1":
                self.pub_au.publish(Bool(data=True))
                if self.is_started and self.script_thread is not None:
                    self.get_logger().warn("Stop script from AU")
                    self.stop_script()
            elif msg.data[-1] == "2":
                self.pub_au.publish(Bool(data=True))
            else:
                # self.get_logger().info("AU deactivated")
                self.pub_au.publish(Bool(data=False))

        elif msg.data.startswith("YELLOWSWITCH"):
            self.synchro_lidar()

        elif msg.data.startswith("BLUESWITCH"):
            self.get_logger().info("Reload Ros")
            os.system('systemctl --user restart launch.service')

        elif msg.data.strip() == "Pos,done":
            self.motion_done = True
            self.motion_done_event.set()

    def robot_data_callback(self, msg: RobotData):
        """Receive the Robot Data from Zynq."""
        self.robot_pos = Position(x=msg.x, y=msg.y, t=msg.theta)
        self.robot_speed = Position(x=msg.vlin, y=msg.vdir, t=msg.vt)

        if not self.motion_done:
            # Update motion state
            self.update_arrival_status()
            self.update_motion_status()

            if self.is_robot_arrived and not self.is_robot_moving:
                self.motion_done = True
                # self.motion_done_event.set()

            elif not self.is_robot_arrived and not self.is_robot_moving:
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
            
            else :
                pass 

    def aruco_callback(self, msg: VisionData):
        if self.robot_pos is not None:
            x_tag = msg.x
            y_tag = msg.y
            self.x_tag = x_tag * np.cos(self.robot_pos.t) - y_tag * np.sin(self.robot_pos.t) + self.robot_pos.x
            self.y_tag = x_tag * np.sin(self.robot_pos.t) + y_tag * np.cos(self.robot_pos.t) + self.robot_pos.y

            best_match_id = None
            min_distance = float('inf')

            for crate_id, crate_info in self.position_crates.items():
                crate_x = crate_info[1]
                crate_y = crate_info[2]
                distance = np.sqrt((self.x_tag - crate_x) ** 2 + (self.y_tag - crate_y) ** 2)
                if distance < min_distance:
                    min_distance = distance
                    best_match_id = crate_id

            if best_match_id is not None and min_distance < 0.002:  # Threshold for matching                
                self.position_crates[best_match_id][1] = self.x_tag  # Update X with tag detection
                self.position_crates[best_match_id][2] = self.y_tag  # Update Y with tag detection
                self.position_crates[best_match_id][3] = msg.theta  # Update Theta with tag detection
                self.position_crates[best_match_id][4] = time.time() - self.match_time  # Update the time of detection from timer_match

            else: # Replace the crate who hasn't been detected for the longest time
                oldest_time = float('inf')
                oldest_crate_id = None

                # for crate_id, crate_info in self.position_crates.items():
                #     if crate_info[0] == 0:  # Only consider crates that are not already matched
                #         if crate_info[4] < oldest_time:
                #             oldest_time = crate_info[4]
                #             oldest_crate_id = crate_id

                if oldest_crate_id is not None:
                    self.position_crates[oldest_crate_id][1] = self.x_tag  # Update X with tag detection
                    self.position_crates[oldest_crate_id][2] = self.y_tag  # Update Y with tag detection
                    self.position_crates[oldest_crate_id][3] = msg.theta  # Update Theta with tag detection
                    self.position_crates[oldest_crate_id][4] = time.time() - self.match_time  # Update the time of detection from timer_match    
            
            # --- Section Logging ---
            home_dir = os.path.expanduser('~')
            log_filename = os.path.join(home_dir, "position_crates_log.json")
            try:
                with open(log_filename, 'w') as f:
                    # indent=4 rend le fichier facile à lire pour un humain
                    json.dump(self.position_crates, f, indent=4)

            except Exception as e:
                self.get_logger().error(f"Erreur lors de l'écriture du log : {e}")

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
                    time.sleep(0.1)
                else:
                    self.send_raw("BLOCK")
                    time.sleep(0.1)

    # def follow_tag_aruco(self):
    #     if not self.stop:
    #         while True:
    #             if self.robot_pos is not None and self.x_tag is not None:
    #                 # 1. Calcul de la distance réelle actuelle au tag
    #                 current_dist = np.sqrt(self.x_tag**2 + self.y_tag**2)
# 
    #                 # 2. Sécurité : on ne bouge que si le tag est à plus de 10cm
    #                 if current_dist > 0.10 and self.id_tag == 36:
    #                     offset = 0.15  # 10 cm
    #                     # Ratio pour trouver le point à 10cm en amont sur le même vecteur
    #                     ratio = (current_dist - offset) / current_dist
# 
    #                     x_target_local = self.x_tag * ratio
    #                     y_target_local = self.y_tag * ratio
# 
    #                     # 3. Transformation vers le repère global (votre formule actuelle)
    #                     pos_x = x_target_local * np.cos(self.robot_pos.t) - y_target_local * np.sin(self.robot_pos.t) + self.robot_pos.x
    #                     pos_y = x_target_local * np.sin(self.robot_pos.t) + y_target_local * np.cos(self.robot_pos.t) + self.robot_pos.y
# 
    #                     # Orientation : on garde l'angle pour faire face au tag
    #                     target_angle = (self.robot_pos.t + np.arctan2(self.y_tag, self.x_tag))
# 
    #                     if self.new_pos > 5: # Votre condition de rafraîchissement
    #                         self.new_pos = 1
    #                         self.send_raw("VMAX 0.5")
    #                         self.move_to(Position(pos_x, pos_y, target_angle))
# 
    #                         self.get_logger().info(f"Cible : {pos_x:.2f}, {pos_y:.2f} (10cm avant tag)")
# 
    #                 time.sleep(0.1)
    #             else:
    #                 self.send_raw("BLOCK")
    #                 time.sleep(0.1)

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
            self.motion_done = False
            self.is_robot_moving = True
            self.is_robot_arrived = False
            time.sleep(0.1)
            self.pub_command.publish(String(data=f"MOVE {pos.x} {pos.y} {pos.t}"))
            self.pos_obj = Position(x=pos.x, y=pos.y, t=pos.t)
            self.motion_done_event.clear()  # Block the wait

    def update_arrival_status(self):
        if not self.stop:
            time.sleep(0.1)
            delta_x = abs(self.pos_obj.x - self.robot_pos.x)
            delta_y = abs(self.pos_obj.y - self.robot_pos.y)
            delta_t = abs(self.pos_obj.t - self.robot_pos.t)
            if delta_x < 0.5 and delta_y < 0.5 and delta_t < 0.5:
                self.is_robot_arrived = True

    def update_motion_status(self):
        if not self.stop:
            time.sleep(0.1)
            if not self.is_robot_moving and not self.motion_done:
                if abs(self.robot_speed.x) < 0.0001 and abs(self.robot_speed.t) < 0.0001:
                    self.is_robot_moving = False

    def wait_for_motion(self):
        """Compute the wait_for_motion action."""
        if not self.stop:
            self.motion_done_event.wait()
            time.sleep(0.2)
            self.motion_done = True

    def servo(self, servo: SERVO_struct):
        """Compute the servo action."""
        if not self.stop:
            self.pub_command.publish(String(data=f"SERVO {servo.servo_id} "
                                                 f"{servo.angle}"
                                            )
                                     )

            time.sleep(0.1)

    def pump(self, pump: PUMP_struct):
        """Compute the pump action."""
        if not self.stop:
            self.pub_command.publish(String(data=f"PUMP {pump.pump_id} "
                                                 f"{pump.enable}"
                                            )
                                     )
            time.sleep(0.1)

    def vaccumgripper(self, vg: VACCUMGRIPPER_struct):
        """Compute the pump action."""
        if not self.stop:
            self.pub_command.publish(String(data=f"VACCUMGRIPPER {vg.vg_id} "
                                                 f"{vg.enable}"
                                            )
                                     )
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
            time.sleep(0.1)

    def valve(self, valve: VALVE_struct):
        """Compute the valve action."""
        if not self.stop:
            self.pub_command.publish(String(data=f"VALVE {valve.valve_id} 1"))

            time.sleep(0.1)

    def write_log(self, message):
        """Write logs."""
        if not self.stop:
            pass

    def sleep(self, duration):
        """Sleep for a given duration."""
        if not self.stop:
            time.sleep(duration)

    def send_raw(self, raw_command):
        """Send raw commands."""
        if not self.stop:
            self.pub_command.publish(String(data=raw_command))
            time.sleep(0.1)

    def kalman(self, kalman: bool):
        """Compute the kalman action."""
        if not self.stop:
            if kalman:
                self.pub_command.publish(String(data="ENKALMAN 1"))
            else:
                self.pub_command.publish(String(data="ENKALMAN 0"))
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
            self.pub_command.publish(String(data=f"{command}"))

    def add_score(self, score):
        """Update the score."""
        if not self.stop:
            self.pub_score.publish(Int32(data=score))
            time.sleep(0.1)

    def sign(self, val):
        return -1 if val < 0 else 1

    def smart_moves(self):
        default_angle = -2.60
        tol = 0.3
        vmax = 0.5
        vtmax = 1.0
        if self.color.lower() == "yellow":
            en_color = 0
        elif self.color.lower() == "blue":
            en_color = 1
        else:
            self.get_logger().error(f"NO COLOR")
        while True:
            self.send_raw(f"VMAX {vmax}")
            self.send_raw(f"VTMAX {vtmax}")
            max = -10
            crate_valid = None
            ind_valid = None
            for ind, crates in self.position_crates.items():
                if self.available_crates[ind]:
                    temp = self.compute_penality(crates)
                    if temp > max:
                        max = temp
                        ind_valid = ind
                        crate_valid = crates
            if crate_valid is not None:
                if crate_valid[2] % 2 == 0:
                    # HERE GO IN FRONT OF crates THAT ARE BORDERLINE
                    self.move_to(Position(crate_valid[0] + (crate_valid[2] - 1) * tol, crate_valid[1], crate_valid[2] * np.pi / 2 + default_angle))
                    self.wait_for_motion()
                    fpos = self.find_final_pos(ind_valid, en_color)
                    self.take_crates(crate_valid[2])
                    self.drop_crates(fpos, default_angle)
                else: # CANS THAT ARE FRONT ON BOARD
                    if self.robot_pos.y > crate_valid[1] or crate_valid[1] < 0.5: # CHECK IF ROBOT ABOVE THE CANS OR CANS CLOSE TO BOUNDARIES
                        if self.robot_pos.y - crate_valid[1] < tol:
                            self.move_to(Position(crate_valid[0] + self.sign(self.robot_pos.x - crate_valid[0]) * tol, crate_valid[1] + tol, 3 * np.pi / 2 + default_angle))
                            self.wait_for_motion()
                        self.move_to(Position(crate_valid[0], crate_valid[1] + tol, 3 * np.pi / 2 + default_angle))
                        self.wait_for_motion()
                        fpos = self.find_final_pos(ind_valid, en_color)
                        self.take_crates(3)
                        self.drop_crates(fpos, default_angle)
                    else:
                        if crate_valid[1] - self.robot_pos.y < tol:
                            self.move_to(Position(crate_valid[0] + self.sign(self.robot_pos.x - crate_valid[0]) * tol, crate_valid[1] - tol, np.pi / 2 + default_angle))
                            self.wait_for_motion()
                        self.move_to(Position(crate_valid[0], crate_valid[1] - tol, np.pi / 2 + default_angle))
                        self.wait_for_motion()
                        fpos = self.find_final_pos(ind_valid, en_color)
                        self.take_crates(1)
                        self.drop_crates(fpos, default_angle)
                self.available_crates[ind_valid] = False
                self.available_end[self.dest_crates[ind_valid][en_color]] += 1 # If yellow 0, else blue
                self.add_score(4)
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

    def smart_moves_2025(self):
        default_angle = -2.60
        tol = 0.3
        vmax = 0.5
        vtmax = 1.0
        if self.color.lower() == "yellow":
            en_color = 0
        elif self.color.lower() == "blue":
            en_color = 1
        else:
            self.get_logger().error(f"NO COLOR")
        while True:
            self.send_raw(f"VMAX {vmax}")
            self.send_raw(f"VTMAX {vtmax}")
            max = -10
            crate_valid = None
            ind_valid = None
            for ind, cans in self.position_crates.items():
                if self.available_crates[ind]:
                    temp = self.compute_penality(cans)
                    if temp > max:
                        max = temp
                        ind_valid = ind
                        crate_valid = cans
            if crate_valid is not None:
                if crate_valid[2] % 2 == 0:
                    # HERE GO IN FRONT OF CANS THAT ARE BORDERLINE
                    self.move_to(Position(crate_valid[0] + (crate_valid[2] - 1) * tol, crate_valid[1], crate_valid[2] * np.pi / 2 + default_angle))
                    self.wait_for_motion()
                    fpos = self.find_final_pos(ind_valid, en_color)
                    self.take_cans(crate_valid[2])
                    self.drop_cans(fpos, default_angle)
                else: # CANS THAT ARE FRONT ON BOARD
                    if self.robot_pos.y > crate_valid[1] or crate_valid[1] < 0.5: # CHECK IF ROBOT ABOVE THE CANS OR CANS CLOSE TO BOUNDARIES
                        if self.robot_pos.y - crate_valid[1] < tol:
                            self.move_to(Position(crate_valid[0] + self.sign(self.robot_pos.x - crate_valid[0]) * tol, crate_valid[1] + tol, 3 * np.pi / 2 + default_angle))
                            self.wait_for_motion()
                        self.move_to(Position(crate_valid[0], crate_valid[1] + tol, 3 * np.pi / 2 + default_angle))
                        self.wait_for_motion()
                        fpos = self.find_final_pos(ind_valid, en_color)
                        self.take_crates(3)
                        self.drop_crates(fpos, default_angle)
                    else:
                        if crate_valid[1] - self.robot_pos.y < tol:
                            self.move_to(Position(crate_valid[0] + self.sign(self.robot_pos.x - crate_valid[0]) * tol, crate_valid[1] - tol, np.pi / 2 + default_angle))
                            self.wait_for_motion()
                        self.move_to(Position(crate_valid[0], crate_valid[1] - tol, np.pi / 2 + default_angle))
                        self.wait_for_motion()
                        fpos = self.find_final_pos(ind_valid, en_color)
                        self.take_crates(1)
                        self.drop_crates(fpos, default_angle)
                self.available_crates[ind_valid] = False
                self.available_end[self.dest_crates[ind_valid][en_color]] += 1 # If yellow 0, else blue
                self.add_score(4)
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
        size_crates = 0.1
        size_robot = 0.27
        pos_out = self.end_poses[self.dest_crates[index][en_color]] # If yellow 0
        incr = self.available_end[self.dest_crates[index][en_color]]
        if pos_out[2] % 2 == 0:
            return [pos_out[0] + (pos_out[2] - 1) * (size_crates / 2 + size_robot + size_crates * incr), pos_out[1], pos_out[2]]
        else:
            return [pos_out[0], pos_out[1] + size_crates / 2 + size_robot + size_crates * incr, 3 * pos_out[2]]

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

    def take_crates(self, angle):
        self.kalman(False)
        self.vaccumgripper(VACCUMGRIPPER_struct(0, 1))
        self.vaccumgripper(VACCUMGRIPPER_struct(1, 1))
        self.vaccumgripper(VACCUMGRIPPER_struct(2, 1))
        self.vaccumgripper(VACCUMGRIPPER_struct(3, 1))
        self.vaccumgripper(VACCUMGRIPPER_struct(4, 1))
        self.vaccumgripper(VACCUMGRIPPER_struct(5, 1))
        self.vaccumgripper(VACCUMGRIPPER_struct(6, 1))
        self.vaccumgripper(VACCUMGRIPPER_struct(7, 1))
        self.vaccumgripper(VACCUMGRIPPER_struct(8, 1))
        self.vaccumgripper(VACCUMGRIPPER_struct(9, 1))
        self.vaccumgripper(VACCUMGRIPPER_struct(10, 1))
        self.vaccumgripper(VACCUMGRIPPER_struct(11, 1))
        self.sleep(0.1)
        push_dst = 0.15
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

    def drop_crates(self, destination, default_angle):
        push_dst = 0.1
        self.move_to(Position(destination[0], destination[1], destination[2] * np.pi / 2 + default_angle))
        self.wait_for_motion()

        self.valve(VALVE_struct(2))
        if destination[2] == 0:
            self.relative_move_to(Position(push_dst, 0, 0))
            self.wait_for_motion()
            self.vaccumgripper(VACCUMGRIPPER_struct(0, 0))
            self.vaccumgripper(VACCUMGRIPPER_struct(1, 0))
            self.vaccumgripper(VACCUMGRIPPER_struct(2, 0))
            self.vaccumgripper(VACCUMGRIPPER_struct(3, 0))
            self.vaccumgripper(VACCUMGRIPPER_struct(4, 0))
            self.vaccumgripper(VACCUMGRIPPER_struct(5, 0))
            self.vaccumgripper(VACCUMGRIPPER_struct(6, 0))
            self.vaccumgripper(VACCUMGRIPPER_struct(7, 0))
            self.vaccumgripper(VACCUMGRIPPER_struct(8, 0))
            self.vaccumgripper(VACCUMGRIPPER_struct(9, 0))
            self.vaccumgripper(VACCUMGRIPPER_struct(10, 0))
            self.vaccumgripper(VACCUMGRIPPER_struct(11, 0))
            self.relative_move_to(Position(-push_dst, 0, 0))
            self.wait_for_motion()
        elif destination[2] == 2:
            self.relative_move_to(Position(-push_dst, 0, 0))
            self.wait_for_motion()
            self.vaccumgripper(VACCUMGRIPPER_struct(0, 0))
            self.vaccumgripper(VACCUMGRIPPER_struct(1, 0))
            self.vaccumgripper(VACCUMGRIPPER_struct(2, 0))
            self.vaccumgripper(VACCUMGRIPPER_struct(3, 0))
            self.vaccumgripper(VACCUMGRIPPER_struct(4, 0))
            self.vaccumgripper(VACCUMGRIPPER_struct(5, 0))
            self.vaccumgripper(VACCUMGRIPPER_struct(6, 0))
            self.vaccumgripper(VACCUMGRIPPER_struct(7, 0))
            self.vaccumgripper(VACCUMGRIPPER_struct(8, 0))
            self.vaccumgripper(VACCUMGRIPPER_struct(9, 0))
            self.vaccumgripper(VACCUMGRIPPER_struct(10, 0))
            self.vaccumgripper(VACCUMGRIPPER_struct(11, 0))
            self.relative_move_to(Position(push_dst, 0, 0))
            self.wait_for_motion()
        elif destination[2] == 3:
            self.relative_move_to(Position(0, -push_dst, 0))
            self.wait_for_motion()
            self.vaccumgripper(VACCUMGRIPPER_struct(0, 0))
            self.vaccumgripper(VACCUMGRIPPER_struct(1, 0))
            self.vaccumgripper(VACCUMGRIPPER_struct(2, 0))
            self.vaccumgripper(VACCUMGRIPPER_struct(3, 0))
            self.vaccumgripper(VACCUMGRIPPER_struct(4, 0))
            self.vaccumgripper(VACCUMGRIPPER_struct(5, 0))
            self.vaccumgripper(VACCUMGRIPPER_struct(6, 0))
            self.vaccumgripper(VACCUMGRIPPER_struct(7, 0))
            self.vaccumgripper(VACCUMGRIPPER_struct(8, 0))
            self.vaccumgripper(VACCUMGRIPPER_struct(9, 0))
            self.vaccumgripper(VACCUMGRIPPER_struct(10, 0))
            self.vaccumgripper(VACCUMGRIPPER_struct(11, 0))
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
