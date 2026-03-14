#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Action Sequencer Node."""

### IDEA SMART SCRPIT: 
# Use stacks and also save solo crates so we can choose the strategy. 
# Use the Actuators ID, so we can manage them 1 by one.
# Actions for pliers
# 0: free, 1: picking, 2: dropping, 3: reverting
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterEvent

from std_msgs.msg import String, Bool, Int32
from opossum_msgs.msg import RobotData, LidarLoc, VisionDataFrame
from std_srvs.srv import Trigger
from types import SimpleNamespace

from scipy.optimize import linear_sum_assignment
import math
import yaml
import numpy as np
from opossum_action_sequencer.utils import (
    Position,
    VACCUMGRIPPER_struct,
    LED_struct,
)

import threading
from threading import Event
import time
import os
import json
from dataclasses import dataclass
from ament_index_python.packages import get_package_share_directory

@dataclass
class Plier:
    """Pliers Class."""

    id: int
    x: float
    y: float
    theta: float
    state: int # -1 nothing, 100 + x, doing action, x have id.
    side_robot: int

    def __init__(self, id, data):
        self.id = id
        self.x = data['x']
        self.y = data['y']
        self.theta = data['t']
        self.state = -1
        self.side_id = id % 4
        self.side_robot = (id // 4) * np.pi / 2

@dataclass
class HazCrate:
    """HazCrate Class."""

    id: int
    x: float
    y: float
    theta: float
    state: int
    color: int
    last_seen: float

    def __init__(self, id, x, y, t, rot = False):
        self.id = id
        self.x = x
        self.y = y
        self.theta = t
        self.state = -1
        self.color = 2 if rot else -1 # -1 don't know, 0 yellow, 1 blue, 2 rot + 100 in zone
        self.last_seen = None

@dataclass
class ZoneRelease:
    id: int
    x: float
    y: float
    state: int

    def __init__(self, id, x, y):
        self.id = id
        self.x = x
        self.y = y
        self.state = []

class ActionManager(Node):
    """Action Manager Node."""

    def __init__(self):
        super().__init__("action_sequencer_node")
        self._init_parameters()
        self._init_publishers()
        self._init_subscribers()
        self._init_services()
        self.last_sent_state = {'crates': {}, 'pliers': {}}
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
        self.pos_obj = None
        self.max_distance = 0.1
        self.latest_camera_msg = None

        # Action Done
        self.is_robot_moving = False
        self.is_robot_arrived = False
        self.is_pump_top_on = False
        self.is_pump_bottom_on = False
        self.robot_pos = None
        self.motion_done = True
        self.color = None
        self._init_mapping()

        # Process
        self.motion_done_event = Event()
        self.motion_done_event.set()

        self.pliers_event = Event()
        self.pliers_event.set()

        self.end_match_event = Event()
        self.end_match_event.set()
        self.stop = False

    def _init_mapping(self):
        objects_path = os.path.join(
            get_package_share_directory("opossum_bringup"), "config", str(self.year), self.config_yaml
        )

        data = yaml.safe_load(open(objects_path, "r"))
        self.pliers = {}
        self.stacks = {}
        self.haz_crates = {}
        self.zones = {}

        for id, act in data['actuators'].items():
            if act['type'] != "vaccum_gripper":
                continue
            plier = Plier(id, act)
            self.pliers[id] = plier

        crates_count = 0
        # Access first stack item
        stacks = data['stacks']
        # Iterate over all map objects
        for id, obj in data['map'].items():
            type_obj = obj['type']
            cos_ = np.cos(obj['t'])
            sin_ = np.sin(obj['t'])
            if "crates" not in type_obj:
                continue
                
            self.stacks[id] = []
            rot = obj['shape'] == 'rot'

            for element in stacks[type_obj].values():
                x = obj['x'] + element['x'] * cos_ - element['y'] * sin_
                y = obj['y'] + element['x'] * sin_ + element['y'] * cos_
                t = obj['t'] + element['t']
                self.haz_crates[crates_count] = HazCrate(crates_count, x, y, t, rot)
                self.stacks[id].append(crates_count)
                crates_count += 1

        for id, zone in data['zones'].items():
            self.zones[id] = ZoneRelease(id, zone['x'], zone['y'])
        
        self.final_zone = data['final_zone']

        # Process objects with cam
        self.barycentre = None
        self.centering = False
        self.list_objects = []
        self.is_center = False
        self.try_center = 0

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
                ("config_yaml", "small_objects.yaml"),
                ("year", 2026),
            ],
        )

        self.year = (
            self.get_parameter("year")
            .get_parameter_value()
            .integer_value
        )
        self.config_yaml = (
            self.get_parameter("config_yaml")
            .get_parameter_value()
            .string_value
        )

    def _init_timers(self):
        """Initialize the timers of the node."""
        self.timer_match = self.create_timer(
            200,
            self.timer_match_callback
        )
        self.timer_backstage = self.create_timer(
            200,
            self.timer_backstage_callback
        )
        self.pub_timer = self.create_timer(0.2, self.publish_board_state)

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

        self.pub_board_state = self.create_publisher(
            String,
            "board_state_updates",
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
            VisionDataFrame,
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

    def _init_services(self):
        """Initialize the services to send full board state on demand."""
        self.full_state_srv = self.create_service(
            Trigger, 
            'get_full_board_state', 
            self.full_state_callback
        )

    def full_state_callback(self, request, response):
        """Service callback to dump the entire board state for map initialization."""
        full_state = {'crates': [], 'pliers': []}

        # Dump all crates
        for cid, crate in self.haz_crates.items():
            full_state['crates'].append({
                'id': cid, 'x': round(crate.x, 3), 'y': round(crate.y, 3),
                'theta': round(crate.theta, 3), 'state': crate.state, 'color': crate.color
            })

        # Dump all pliers
        for pid, plier in self.pliers.items():
            full_state['pliers'].append({
                'id': pid, 'x': round(plier.x, 3), 'y': round(plier.y, 3),
                'theta': round(plier.theta, 3), 'state': plier.state
            })

        # Return it directly in the service response string
        response.success = True
        response.message = json.dumps(full_state)
        
        # Optional: You can also reset last_sent_state here so the next timer 
        # tick forces a full delta update, just to be safe.
        self.last_sent_state = {'crates': {}, 'pliers': {}}
        
        self.get_logger().info("Full board state sent to map via service.")
        return response

    def publish_board_state(self):
        """Publish only the crates and pliers that have changed."""
        # Wait until the map is loaded
        if not hasattr(self, 'haz_crates') or not hasattr(self, 'pliers'):
            return

        updates = {'crates': [], 'pliers': []}

        # 1. Check Crates for changes
        for cid, crate in self.haz_crates.items():
            # Create a tuple of the state we care about (rounded to 3 decimals ~ 1mm)
            current_state = (round(crate.x, 3), round(crate.y, 3), round(crate.theta, 3), crate.state, crate.color)
            last_state = self.last_sent_state['crates'].get(cid)
            
            if current_state != last_state:
                updates['crates'].append({
                    'id': cid, 'x': current_state[0], 'y': current_state[1],
                    'theta': current_state[2], 'state': current_state[3], 'color': current_state[4]
                })
                # Update our memory
                self.last_sent_state['crates'][cid] = current_state

        # 2. Check Pliers for changes
        for pid, plier in self.pliers.items():
            current_state = (round(plier.x, 3), round(plier.y, 3), round(plier.theta, 3), plier.state)
            last_state = self.last_sent_state['pliers'].get(pid)
            
            if current_state != last_state:
                updates['pliers'].append({
                    'id': pid, 'x': current_state[0], 'y': current_state[1],
                    'theta': current_state[2], 'state': current_state[3]
                })
                self.last_sent_state['pliers'][pid] = current_state

        # 3. Only publish if there is actually something new!
        if updates['crates'] or updates['pliers']:
            msg = String()
            msg.data = json.dumps(updates)
            self.pub_board_state.publish(msg)

    # =========================================================================
    # UNIFIED CAMERA CALLBACK (HUNGARIAN TRACKING WITH ARUCO ID)
    # =========================================================================
    def aruco_callback(self, msg: VisionDataFrame):
        """Continuously save the latest camera frame without processing it."""
        self.latest_camera_msg = msg

    def _extract_color_from_id(self, aruco_id: int) -> int:
        """Map ArUco ID to internal color code."""
        if aruco_id == 37:
            return 0  # Yellow
        elif aruco_id == 43:
            return 1  # Blue
        elif aruco_id == 21:
            return 2  # Rot (Red)
        return -1 # Unknown

    def stare_and_update(self):
        """Stop, let the camera settle, and process the latest frame in World Coordinates."""
        if self.stop:
            return

        if getattr(self, 'robot_pos', None) is None:
            self.get_logger().warn("Stare failed: Robot position unknown.")
            return

        self.get_logger().info("Staring... waiting for camera to settle.")
        
        # 1. Wait for physical motion blur to clear
        time.sleep(0.2) 
        
        # 2. Grab the latest message in the buffer
        msg = self.latest_camera_msg
        if msg is None or not msg.object:
            self.get_logger().info("Stare complete: No objects currently visible.")
            return

        self.get_logger().info("Processing camera snapshot...")
        camera_detections = msg.object
        current_time = time.time()

        # =====================================================================
        # 3. TRANSFORM DETECTIONS: ROBOT FRAME -> WORLD FRAME
        # =====================================================================
        world_detections = []
        cos_t = math.cos(self.robot_pos.t)
        sin_t = math.sin(self.robot_pos.t)

        for det in camera_detections:
            world_det = SimpleNamespace()
            world_det.id = det.id
            world_det.x = self.robot_pos.x + (det.x * cos_t - det.y * sin_t)
            world_det.y = self.robot_pos.y + (det.x * sin_t + det.y * cos_t)
            world_det.theta = self.robot_pos.t + det.theta
            world_detections.append(world_det)


        # =====================================================================
        # 4. HUNGARIAN TRACKING LOGIC (Using World Detections)
        # =====================================================================
        if not self.haz_crates:
            for det in world_detections:
                new_id = max(self.haz_crates.keys()) + 1 if self.haz_crates else 0
                color_val = self._extract_color_from_id(det.id)
                new_crate = HazCrate(new_id, det.x, det.y, det.theta, rot=(color_val == 2))
                new_crate.color = color_val
                new_crate.last_seen = current_time
                self.haz_crates[new_id] = new_crate
            return

        crate_ids = list(self.haz_crates.keys())
        num_crates = len(crate_ids)
        num_detections = len(world_detections)
        
        cost_matrix = np.zeros((num_crates, num_detections))

        for i, cid in enumerate(crate_ids):
            crate = self.haz_crates[cid]
            for j, det in enumerate(world_detections):
                # Now we are comparing World to World!
                dist = math.hypot(crate.x - det.x, crate.y - det.y)
                cost_matrix[i, j] = dist

        crate_indices, detection_indices = linear_sum_assignment(cost_matrix)
        matched_detection_indices = set()

        for crate_idx, det_idx in zip(crate_indices, detection_indices):
            distance = cost_matrix[crate_idx, det_idx]

            # max_distance should probably be around 0.10 to 0.15 (10-15cm) to allow for minor camera errors
            if distance <= self.max_distance:
                cid = crate_ids[crate_idx]
                matched_crate = self.haz_crates[cid]
                det = world_detections[det_idx]

                matched_crate.x = det.x
                matched_crate.y = det.y
                matched_crate.theta = det.theta
                matched_crate.last_seen = current_time

                if matched_crate.color in [-1, 2]:
                    matched_crate.color = self._extract_color_from_id(det.id)

                matched_detection_indices.add(det_idx)

        for j, det in enumerate(world_detections):
            if j not in matched_detection_indices:
                new_id = max(self.haz_crates.keys()) + 1
                color_val = self._extract_color_from_id(det.id)
                new_crate = HazCrate(new_id, det.x, det.y, det.theta, rot=(color_val == 2))
                new_crate.color = color_val
                new_crate.last_seen = current_time
                self.haz_crates[new_id] = new_crate
                self.get_logger().info(f"Tracking: Discovered new crate! Assigned internal ID: {new_id} at X:{det.x:.2f} Y:{det.y:.2f}")

        # # =====================================================================
        # # OPTIONAL: 5. GHOST CLEANUP 
        # # (Remove crates that we haven't seen in a while to handle stolen/moved crates)
        # # =====================================================================
        # timeout_seconds = 10.0 # How long until we forget a crate exists
        # stale_crates = []
        # for cid, crate in self.haz_crates.items():
        #     if current_time - crate.last_seen > timeout_seconds:
        #         # Don't delete crates that are currently inside our map stacks or held by pliers
        #         if crate.state == -1 and not self.get_stack_linked(cid):
        #             stale_crates.append(cid)
                    
        # for cid in stale_crates:
        #     del self.haz_crates[cid]
        #     self.get_logger().info(f"Tracking: Removed stale crate ID {cid} from memory.")

    def begin_centering(self):
        self.centering = True
        self.list_objects = []
        self.barycentre = None

    def center_front_stack(self):
        if self.barycentre is not None:
            self.try_center = 0
            self.get_logger().info(f"Barycentre: {self.barycentre}")
            self.relative_move_to(Position(self.barycentre[0], self.barycentre[1], 3.14159 - self.robot_pos.t))
            self.wait_for_motion()

            time.sleep(0.25)
            self.begin_centering()
            time.sleep(1.0)
            if self.barycentre is not None:
                while abs(self.barycentre[0]- 0.22) > 0.005 or abs(self.barycentre[1]) > 0.005 and self.try_center < 4:
                    self.get_logger().info(f"Barycentre: {self.barycentre}")
                    self.relative_move_to(Position(self.barycentre[0], self.barycentre[1], 3.14159 - self.robot_pos.t))
                    self.wait_for_motion()
                    self.try_center += 1
                    time.sleep(0.25)

    def parameter_event_callback(self, event):
        """Handle the parameters event."""
        
        # 1. On récupère le nom absolu du noeud qui a émis l'événement
        sender_node = event.node
        
        # 2. On récupère le namespace actuel de CE noeud (ex: '/main_robot')
        my_namespace = self.get_namespace()
        
        # 3. GUARD CLAUSE : Si l'événement ne vient pas de notre namespace, on l'ignore.
        # On vérifie aussi que notre namespace n'est pas le root '/' pour éviter les bugs.
        if my_namespace != '/' and not sender_node.startswith(my_namespace):
            return

        # Parcours des paramètres modifiés
        script_map = {
            0: ("opossum_action_sequencer.match.script1", "Default Script"),
            1: ("opossum_action_sequencer.match.script1", "Script 1"),
            2: ("opossum_action_sequencer.match.script2", "Script 2"),
            3: ("opossum_action_sequencer.match.script3", "Script 3"),
            4: ("opossum_action_sequencer.match.script4", "Script 4"),
            5: ("opossum_action_sequencer.match.script5", "Script 5"),
            6: ("opossum_action_sequencer.match.script6", "Script 6"),
            7: ("opossum_action_sequencer.match.script_homologation", "Script Homologation"),
            9: ("opossum_action_sequencer.match.follow_ennemi", "Script Follow Ennemi"),
            11: ("opossum_action_sequencer.match.smart_script", "Script Smart"),
            12: ("opossum_action_sequencer.match.smart_script", "Script Smart"),
            69: ("opossum_action_sequencer.match.script_init", "Script Init"),
        }

        # La suite de ton code reste identique
        for changed in event.changed_parameters:
            if changed.name == "script_number":
                script_num = changed.value.integer_value

                if script_num in script_map:
                    module_path, log_msg = script_map[script_num]
                    # self.get_logger().info(f"[{my_namespace}] {log_msg}")

                    # Dynamically import the Script class
                    module = __import__(module_path, fromlist=["Script"])
                    Script = getattr(module, "Script")

                else:
                    self.get_logger().error(f"[{my_namespace}] Aucun script associé à la valeur : {script_num}")
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
        self.color = 0 if msg.data.lower() == "yellow" else 1
        self.final_zone = self.final_zone[self.color]

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

        elif msg.data.startswith("PINCEFEEDBACK"):
            data = msg.data.split()[1:]
            id = int(data[0])
            # action = int(data[1])
            s0 = int(data[2])
            s1 = int(data[3])
            plier0 = self.pliers[id  * 2]
            plier1 = self.pliers[id  * 2 + 1]
            # for pl in self.pliers.values():
            #     self.get_logger().info(f"Before state: {pl.state}")
            if plier0.state >= 99:
                if s0 == 1:
                    plier0.state -= 100
                else:
                    plier0.state = -2
            
            if plier1.state >= 99:
                if s1 == 1:
                    plier1.state -= 100
                else:
                    plier1.state = -2
            # for pl in self.pliers.values():
            #     self.get_logger().info(f"state: {pl.state}")
            if all(pl.state < 99 for pl in self.pliers.values()):
                self.pliers_event.set()

    def robot_data_callback(self, msg: RobotData):
        """Receive the Robot Data from Zynq."""
        self.robot_pos = Position(x=msg.x, y=msg.y, t=msg.theta)
        self.robot_speed = Position(x=msg.vlin, y=msg.vdir, t=msg.vt)

        if not self.motion_done:
            # Update motion state
            self.update_arrival_status()
            is_stopped = abs(self.robot_speed.x) < 0.02 and abs(self.robot_speed.t) < 0.02

            # Si on est proche de la cible ET à l'arrêt, on débloque le script.
            if self.is_robot_arrived and is_stopped:
                # self.get_logger().info("Arrivée confirmée par calcul (Timeout logiciel)")
                self.motion_done = True
                self.motion_done_event.set() # C'est ça qui débloque wait_for_motion

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

    def relative_move_to(self, delta: Position, seuil=0.1):
        """Compute the relative move_to action with frame transformation (Robot -> Board)."""
        if not self.stop:
            self.timer = None
            self.seuil = seuil
            
            # 1. Récupération de l'angle actuel du robot (en radians)
            theta = self.robot_pos.t
            
            # 2. Rotation du vecteur delta (repère robot) vers le repère plateau
            # On projette le déplacement relatif selon l'orientation actuelle
            delta_x_plateau = (delta.x - 0.22) * np.cos(theta) - delta.y * np.sin(theta)
            delta_y_plateau = (delta.x - 0.22) * np.sin(theta) + delta.y * np.cos(theta)
            
            # 3. Calcul de la position cible finale dans le repère plateau
            pos = Position(
                x = self.robot_pos.x + delta_x_plateau,
                y = self.robot_pos.y + delta_y_plateau,
                t = self.robot_pos.t + delta.t  # L'angle reste une simple addition
            )
            
            self.motion_done = False
            self.is_robot_moving = True
            self.is_robot_arrived = False
            
            # Petit temps de pause pour laisser le système respirer si nécessaire
            time.sleep(0.1)
            
            # Envoi de la commande
            self.pub_command.publish(String(data=f"MOVE {pos.x} {pos.y} {pos.t}"))
            
            # Mise à jour de l'objectif
            self.pos_obj = pos
            self.motion_done_event.clear()

    def update_arrival_status(self):
        if not self.stop and self.pos_obj is not None:
            dist = np.sqrt((self.pos_obj.x - self.robot_pos.x)**2 + 
                           (self.pos_obj.y - self.robot_pos.y)**2)

            # Seuil de 2cm (0.02) et environ 3 degrés (0.05 rad)
            if dist < 0.02 and abs(self.pos_obj.t - self.robot_pos.t) < 0.05:
                self.is_robot_arrived = True
            else:
                self.is_robot_arrived = False

    def wait_for_motion(self, timeout=10):
        """Compute the wait_for_motion action."""
        if not self.stop:
            self.motion_done_event.wait(timeout=timeout) 
            self.motion_done = True

    def wait_for_plier(self):
        """Compute the wait_for_motion action."""
        if not self.stop:
            self.pliers_event.wait()

    def send_plier_cmd(self, ids: dict):
        """Compute and send the vacuum gripper command with pair optimization.
        
        ids: dict containing the plier ID as key and the assigned action string as value.
             Example: {2: "pick", 7: "drop", 8: "rev_drop", 0: "pick"}
             
        Modes:
        1: pick
        2: drop
        3: rev_drop
        4: rev_drop for the first, drop for the second
        5: drop for the first, rev_drop for the second
        """
        if self.stop or not ids:
            return

        self.pliers_event.clear()
        
        # Map string actions to their base modes
        action_map = {
            "pick": 1,
            "drop": 2,
            "rev_drop": 3
        }

        # 1. Trier les clés (IDs) pour faciliter le groupement
        sorted_ids = sorted(ids.keys())
        processed_ids = set()

        for current_id in sorted_ids:
            if current_id in processed_ids:
                continue

            cmd_id = current_id // 2
            pair_neighbor = current_id + 1
            action1 = ids[current_id]

            # Vérifier si c'est un ID pair et si son voisin direct est aussi commandé
            if current_id % 2 == 0 and pair_neighbor in ids:
                action2 = ids[pair_neighbor]
                
                # Cas A : Les deux pinces ont la même action (ex: pick/pick)
                if action1 == action2:
                    mode = action_map.get(action1, 1) # Fallback to 1 if unknown string
                    self.pub_command.publish(String(data=f"PINCE {cmd_id} {mode} 2"))
                    self.get_logger().info(f"PUB HERE 0")
                    processed_ids.add(current_id)
                    processed_ids.add(pair_neighbor)
                    continue
                
                # Cas B : Combinaison mixte spéciale -> rev_drop(0) et drop(1)
                elif action1 == "rev_drop" and action2 == "drop":
                    self.pub_command.publish(String(data=f"PINCE {cmd_id} 4 2"))
                    self.get_logger().info(f"PUB HERE 1")
                    processed_ids.add(current_id)
                    processed_ids.add(pair_neighbor)
                    continue
                
                # Cas C : Combinaison mixte spéciale -> drop(0) et rev_drop(1)
                elif action1 == "drop" and action2 == "rev_drop":
                    self.pub_command.publish(String(data=f"PINCE {cmd_id} 5 2"))
                    self.get_logger().info(f"PUB HERE 2")
                    processed_ids.add(current_id)
                    processed_ids.add(pair_neighbor)
                    continue
                
                # Si actions mixtes non supportées par paire (ex: pick + drop),
                # on laisse le code continuer pour les traiter individuellement.

            # Traitement individuel (ID seul, voisin manquant, ou paire incompatible)
            mode = action_map.get(action1, 1)
            side = current_id % 2
            self.pub_command.publish(String(data=f"PINCE {cmd_id} {mode} {side}"))
            self.get_logger().info(f"PUB HERE 4")
            processed_ids.add(current_id)

    def vaccumgripper(self, vg: VACCUMGRIPPER_struct):
        """Compute the pump action."""
        if not self.stop:
            self.pub_command.publish(String(data=f"PINCE {vg.id} {vg.mode} {vg.side}"))
            time.sleep(0.1)

    def led(self, led: LED_struct):
        """Compute the led action."""
        self.pub_command.publish(String(data=f"LED {led.red} "
                                             f"{led.green} {led.blue}")
                                 )

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
        self.send_raw(f"VMAX 0.5")
        self.send_raw(f"VTMAX 1.5")
        release = False
        while True:

            best_ind, stack_id, is_inv = self.compute_pick_rewards()
            if release:
                best_zone_ind = self.compute_release_rewards()
                if best_zone_ind is not None:
                    best_zone = self.zones[best_zone_ind]
                    dx = self.robot_pos.x - best_zone.x
                    dy = self.robot_pos.y - best_zone.y
                    distance = 0.15
                    if dx > dy:
                        if dx > 0:
                            fpos = Position(best_zone.x + distance, best_zone.y, 0.0)
                            angle = 3.14
                        else:
                            fpos = Position(best_zone.x - distance, best_zone.y, 0.0)
                            angle = 0
                    else:
                        if dy > 0:
                            fpos = Position(best_zone.x, best_zone.y + distance, 0.0)
                            angle = 4.71
                        else:
                            fpos = Position(best_zone.x, best_zone.y - distance, 0.0)
                            angle = 1.57
                    id_side = self.get_best_side_pliers_release(angle)
                    if id_side is not None:
                        fpos.t = angle - self.pliers[id_side * 4].theta
                        self.move_to(fpos)
                        self.wait_for_motion()

                        id_active_pliers = {id: "drop" if self.haz_crates[self.pliers[id].state].color == self.color else "rev_drop" for id in self.pliers.keys() if (self.pliers[id].state >= 0 and id // 4 == id_side)}
                        for id in id_active_pliers.keys():
                            pstate = self.pliers[id].state
                            if pstate >= 0:
                                self.haz_crates[pstate].state = 100
                                self.zones[best_zone_ind].state.append(pstate)

                        self.send_plier_cmd(id_active_pliers)
                        self.wait_for_plier()

                        # Set state of actuators
                        for pl_id in id_active_pliers:
                            self.pliers[pl_id].state = -1
                    else: 
                        self.get_logger().info("Issue cannot find anymore plier taken")
                        release = False
                else: 
                    self.get_logger().info("Issue cannot find")
                    self.move_to(Position(self.final_zone["x"], self.final_zone["y"], 0.0))
                    self.wait_for_motion()
                    break
            # Get the best crate to take, according to the color / positions... 
            elif best_ind is not None:
                # Depending on the stack len (or alone) we check available pliers
                # A. Déterminer le groupe d'objets (Stack ou Single)
                target_ids = self.stacks[stack_id] if stack_id is not None else [best_ind]

                # B. Trouver les pinces disponibles
                pliers_config = self.get_best_side_pliers(len(target_ids))
                if not pliers_config:
                    self.get_logger().warn("Cible trouvée mais aucune pince disponible. Passage en mode Release.")
                    release = True
                    continue
                
                selected_pliers_ids = pliers_config[0]
    
                # C. Calculer les centres de masse (Crates et Pliers)
                target_pos = self.get_mean_pose([self.haz_crates[pid] for pid in target_ids])
                pliers_pos = self.get_mean_pose([self.pliers[pid] for pid in selected_pliers_ids])

                # D. Étape 1 : Navigation vers le point d'entrée
                # On s'aligne face à la caisse. On ajoute pi car le robot "recule" ou "fait face" ?
                # Si le robot doit faire face, l'angle est target_pos.theta
                

                # E. Étape 2 : Alignement final (Calcul de l'offset)
                # L'angle final du robot doit être tel que les pinces soient alignées avec la caisse
                # target_angle = Angle_Caisse - Angle_Pince_dans_Robot
                # Si is_inv est True, on attaque par l'autre côté (+ pi)
                target_angle = target_pos.t
                if is_inv:
                    target_angle += np.pi
                final_robot_theta = target_angle - pliers_pos.t

                # Transformer la position de la pince du repère Robot vers Map
                x_offset, y_offset = self.rotate_point(pliers_pos.x, pliers_pos.y, final_robot_theta)
                
                distance = 0.15
                entry_pos = Position(
                    target_pos.x - x_offset - distance * np.cos(target_angle),
                    target_pos.y - y_offset - distance * np.sin(target_angle),
                    final_robot_theta
                )
                # Position finale du robot = Position Caisse - Offset Pince
                final_pos = Position(
                    target_pos.x - x_offset,
                    target_pos.y - y_offset,
                    final_robot_theta
                )

                self.move_to(entry_pos)
                self.wait_for_motion()
                self.stare_and_update()

                self.move_to(final_pos)
                self.wait_for_motion()

                # F. Activation des pinces
                robot_x, robot_y, robot_t = final_pos.x, final_pos.y, final_pos.t

                # 2. Projeter chaque pince sélectionnée dans la Map
                pliers_in_map = {}
                for pl_id in selected_pliers_ids:
                    p_local = self.pliers[pl_id]
                    # Rotation de la position de la pince (Robot -> Map)
                    rx, ry = self.rotate_point(p_local.x, p_local.y, robot_t)
                    # Translation (Ajout de la position du robot)
                    px_map = robot_x + rx
                    py_map = robot_y + ry
                    pliers_in_map[pl_id] = (px_map, py_map)

                # 3. Associer chaque pince à l'objet le plus proche (Nearest Neighbor)
                remaining_targets = list(target_ids) # Copie des IDs d'objets dans la pile

                for pl_id, (px, py) in pliers_in_map.items():
                    if not remaining_targets:
                        break
                        
                    # Trouver l'objet le plus proche de CETTE pince spécifique
                    best_obj_id = min(
                        remaining_targets, 
                        key=lambda obj_id: (self.haz_crates[obj_id].x - px)**2 + (self.haz_crates[obj_id].y - py)**2
                    )
                    
                    # Retirer l'objet de la liste pour ne pas l'assigner deux fois
                    remaining_targets.remove(best_obj_id)
                    
                    # 4. Appliquer le State 100 + ID
                    new_state = 100 + best_obj_id
                    self.pliers[pl_id].state = new_state
                    self.haz_crates[best_obj_id].state = 1
                
                dict_sel_pliers = {id: "pick" for id in selected_pliers_ids}
                # 5. Envoyer la commande
                self.send_plier_cmd(dict_sel_pliers)
                # Set state of objects
                if stack_id is not None:
                    for haz_id in self.stacks[stack_id]:
                        self.haz_crates[haz_id].state = 1
                else:
                    self.haz_crates[best_ind].state = 1
                
                # Set state of actuators
                self.wait_for_plier()
                continue
            else:
                self.get_logger().info("Nothing to do, going for the release position")
                release = True

    def get_mean_pose(self, objects):
        """Calcule la position moyenne (x, y, theta_circulaire) d'une liste d'objets."""
        mx = np.mean([obj.x for obj in objects])
        my = np.mean([obj.y for obj in objects])
        m_sin = np.mean([np.sin(obj.theta) for obj in objects])
        m_cos = np.mean([np.cos(obj.theta) for obj in objects])
        return Position(mx, my, np.arctan2(m_sin, m_cos))

    def rotate_point(self, x, y, theta):
        """Applique une rotation 2D simple."""
        cos_t, sin_t = np.cos(theta), np.sin(theta)
        rx = x * cos_t - y * sin_t
        ry = x * sin_t + y * cos_t
        return rx, ry

    def compute_release_rewards(self):
        max_reward = float('-inf')
        best_zone_id = None
        for id, zone in self.zones.items():
            if len(zone.state) > 0:
                continue
            reward = self.compute_release_penality(zone.x, zone.y)

            # 5. Mise à jour du champion global
            if reward > max_reward:
                max_reward = reward
                best_zone_id = id
        return best_zone_id

    def compute_release_penality(self, x, y):
        coeff_center = -10 # -0.005
        coeff_dst = 0
        coeff_enn = 0 # 0.05
        # coeef_end = -0.0001
        val_center = (1.5 - x) ** 2 + (1 - y) ** 2
        val_dst = (self.robot_pos.x - x) ** 2 + (self.robot_pos.y - y) ** 2
        if self.x_enn is not None:
            val_ennemi = (self.x_enn - x) ** 2 + (self.y_enn - y) ** 2
        else:
            val_ennemi = 0
        self.get_logger().info(f"For {x}, {y}, center: {val_center}, coeff_dst: {coeff_dst}, total: {coeff_dst * val_dst + coeff_enn * val_ennemi + coeff_center * val_center}")
        return coeff_dst * val_dst + coeff_enn * val_ennemi + coeff_center * val_center

    def compute_pick_penality(self, x, y):
        coeff_center = -1 # -0.005
        coeff_dst = -1
        coeff_enn = -10 # 0.05
        # coeef_end = -0.0001
        val_center = (1.5 - x) ** 2 + (1 - y) ** 2
        val_dst = (self.robot_pos.x - x) ** 2 + (self.robot_pos.y - y) ** 2
        if self.x_enn is not None:
            val_ennemi = (self.x_enn - x) ** 2 + (self.y_enn - y) ** 2
        else:
            val_ennemi = 0
        return coeff_dst * val_dst + coeff_enn * val_ennemi + coeff_center * val_center

    def compute_pick_rewards(self):
        """
        Calcule la meilleure opportunité de ramassage et retourne l'identifiant, 
        le stack, l'orientation d'approche et les coordonnées du point d'entrée.
        """
        max_reward = float('-inf')
        best_crate_id = None
        target_stack_id = None
        use_inverted_approach = False
        
        approach_distance = 0.25
        reviewed_ids = set()

        for crate_id, crate in self.haz_crates.items():
            if crate_id in reviewed_ids:
                continue

            # 1. Filtres de sécurité et d'état
            if (crate.color == self.color or crate.color == 2) and crate.state == 100:
                reviewed_ids.add(crate_id)
                continue
            if crate.state == 1 or crate.state == 100: 
                reviewed_ids.add(crate_id)
                continue
            
            # 2. Consolidation des données (Caisse seule ou Pile)
            stack_id = self.get_stack_linked(crate_id)
            if stack_id is not None:
                group_ids = self.stacks[stack_id]
                group_crates = [self.haz_crates[pid] for pid in group_ids]
                mean_x = np.mean([c.x for c in group_crates])
                mean_y = np.mean([c.y for c in group_crates])
                
                # Moyenne circulaire des angles
                sum_sin = np.sum([np.sin(c.theta) for c in group_crates])
                sum_cos = np.sum([np.cos(c.theta) for c in group_crates])
                mean_theta = np.arctan2(sum_sin, sum_cos)
                
                for pid in group_ids:
                    reviewed_ids.add(pid)
            else:
                mean_x, mean_y, mean_theta = crate.x, crate.y, crate.theta
                reviewed_ids.add(crate_id)

            # 3. Calcul de l'orientation d'approche perpendiculaire
            offset_x = np.cos(mean_theta) * approach_distance
            offset_y = np.sin(mean_theta) * approach_distance

            entry_point_std = (mean_x - offset_x, mean_y - offset_y)
            entry_point_inv = (mean_x + offset_x, mean_y + offset_y)

            # 4. Évaluation des points d'entrée
            reward_std = self.compute_pick_penality(*entry_point_std)
            reward_inv = self.compute_pick_penality(*entry_point_inv)

            # Sélection locale du meilleur point
            if reward_inv > reward_std:
                current_local_reward = reward_inv
                current_inverted = True
            else:
                current_local_reward = reward_std
                current_inverted = False
            
            # 5. Mise à jour du champion global
            if current_local_reward > max_reward:
                max_reward = current_local_reward
                best_crate_id = crate_id
                target_stack_id = stack_id
                use_inverted_approach = current_inverted
                            
        return best_crate_id, target_stack_id, use_inverted_approach

    def get_stack_linked(self, target_index):
        """
        Vérifie si un indice est présent dans l'une des listes du dictionnaire.
        
        Args:
            stacks_dict (dict): Dictionnaire de listes {stack_id: [indices...]}
            target_index (int): L'indice à rechercher
            
        Returns:
            int/None: L'ID du stack s'il est trouvé, sinon None.
        """
        for stack_id, indices in self.stacks.items():
            if target_index in indices:
                return stack_id
                
        return None

    def get_best_side_pliers_release(self, angle_target):
        max_reward = float('+inf')
        best_pliers_side = None
        for side in range(4):
            side_pliers_ids = range(side * 4, (side + 1) * 4)
            if any(self.pliers[pl_id].state >= 0 for pl_id in side_pliers_ids):
                rew = self.angular_distance(self.robot_pos.t + self.pliers[side * 4].theta, angle_target)
                if rew < max_reward:
                    max_reward = rew
                    best_pliers_side = side
        return best_pliers_side

    def get_best_side_pliers(self, stack_len):
        """
        Identifie les côtés du robot ayant suffisamment de ventouses libres en ligne.
        
        Args:
            stack_len (int): Nombre d'objets dans la pile à ramasser.
            
        Returns:
            list: Liste des side_ids (0, 1, 2 ou 3) capables de prendre la pile.
        """
        available_sides = []
        
        # Il y a 4 côtés (0, 1, 2, 3)
        for side in range(4):
            # On récupère les 4 pinces de ce côté spécifique
            # Les IDs sont organisés par blocs de 4 : [0,1,2,3], [4,5,6,7], etc.
            side_pliers_ids = range(side * 4, (side + 1) * 4)
            
            max_consecutive = 0
            current_consecutive = 0
            max_list_consecutive = []
            current_list_consecutive = []
            
            for p_id in side_pliers_ids:
                if self.pliers[p_id].state == -1:
                    current_consecutive += 1
                    current_list_consecutive.append(p_id)
                    if current_consecutive > max_consecutive:
                        max_consecutive = current_consecutive
                        max_list_consecutive = current_list_consecutive
                else:
                    current_consecutive = 0 # Rupture de la ligne
                    current_list_consecutive = []            
            # Si le plus grand groupe de ventouses libres est suffisant
            if max_consecutive >= stack_len:
                available_sides.append(max_list_consecutive[:stack_len])
        
        # for side in available_sides:
        #     self.robot_pos.t + self.pliers[side].theta 
        return available_sides if available_sides != [] else None

    def angular_distance(self, a1, a2):
        diff = (a2 - a1 + np.pi) % (2 * np.pi) - np.pi
        return abs(diff)

def main(args=None):
    """Run main loop."""
    rclpy.init(args=args)
    action_manager_node = ActionManager()
    rclpy.spin(action_manager_node)
    action_manager_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
