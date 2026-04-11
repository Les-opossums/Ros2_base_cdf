#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Action Sequencer Node."""

# TODO: Remove ghosts (review does not seem to work)
# TODO: Review the path finder, sometimes the robot goes above known stacks
# TODO: optimize angles for take / release, sometimes does the whoel turn (but dont take in in the cost, just after)
# TODO add a go back to zone before the end
# TODO better handle the cross with an ennemy
# TODO: Associate each plier with a camera.
# Actions for pliers
# 0: free, 1: picking, 2: dropping, 3: reverting

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterEvent
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

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
        self.is_running = False
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
    pick_reward: float
    best_pick_path: list
    use_inverted: bool
    is_part_of_stack: list

    def __init__(self, id, x, y, t, rot = False):
        self.id = id
        self.x = x
        self.y = y
        self.theta = t
        self.state = -1
        self.attempts = 0
        self.color = 2 if rot else -1 # -1 don't know, 0 yellow, 1 blue, 2 rot + 100 in zone
        self.last_seen = None
        self.pick_reward = float('-inf') 
        self.best_pick_path = []
        self.use_inverted = False
        self.is_part_of_stack = [] # Store the IDs of the stack it belongs to

@dataclass
class ZoneRelease:
    id: int
    x: float
    y: float
    size: float
    release_reward: float
    best_release_path: list
    best_release_pos: list

    def __init__(self, id, x, y, size):
        self.id = id
        self.x = x
        self.y = y
        self.size = size
        self.release_reward = float('-inf')
        self.best_release_path = []
        self.best_release_pos = None

class ActionManager(Node):
    """Action Manager Node."""

    def __init__(self):
        super().__init__("action_sequencer_node")
        self.cb_group = ReentrantCallbackGroup()
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
        self.match_finished = False
        self.backstage_sequence = False

        self.x_enn = None
        self.y_enn = None

        self.x_tag = None
        self.y_tag = None

        self.new_pos = 0
        self.pos_obj = None
        self.max_distance = 0.2
        self.latest_camera_msg = {1: None, 2: None, 3: None}
        self.last_camera_timestamp = {1: time.time(), 2: time.time(), 3: time.time()}

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
        self.data_lock = threading.RLock()  # <--- Added the 'R'
        # self.camera_angles = [0.0, 2.093333, 4.186666]
        # self.camera_angles = [0.0]

        # --- NEW: Navigation Constraints ---
        self.robot_radius = 0.18
        
        # Safe boundaries (shrunk by robot radius):
        self.safe_x_min = self.boundaries[0] + self.robot_radius
        self.safe_x_max = self.boundaries[1] - self.robot_radius
        self.safe_y_min = self.boundaries[2] + self.robot_radius
        self.safe_y_max = self.boundaries[3] - self.robot_radius
        
        # Forbidden Zone (Real: x=[0.6, 2.4], y=[1.55, 2.0])
        # Expanded Forbidden Zone (grown by robot radius):
        self.f_zone_x_min = 0.6 - self.robot_radius  # 0.35
        self.f_zone_x_max = 2.4 + self.robot_radius  # 2.65
        self.f_zone_y_min = 1.55 - self.robot_radius # 1.30
        self.f_zone_y_max = 2.0                      # Capped at top
        self.final_zone = None
        self.steal_poses = None

    def is_point_safe(self, x, y):
        """Check if a point is within boundaries and outside forbidden zones."""
        # 1. Check outer boundaries
        if not (self.safe_x_min <= x <= self.safe_x_max and self.safe_y_min <= y <= self.safe_y_max):
            return False
            
        # 2. Check forbidden zone
        if (self.f_zone_x_min <= x <= self.f_zone_x_max) and (self.f_zone_y_min <= y <= self.f_zone_y_max):
            return False
            
        return True

    def is_position_fully_safe(self, x, y, ignored_crate_id=None):
        # 1. Check static forbidden zones (Your existing function)
        if not self.is_point_safe(x, y):
            return False

        # 2. Check dynamic crates (New check)
        # We use 0.33 because: Robot(0.25) + CrateHalfWidth(0.075) + Buffer(0.005)
        for cid, crate in self.haz_crates.items():
            if cid == ignored_crate_id or crate.state != -1:
                continue
            if math.hypot(x - crate.x, y - crate.y) < 0.25:
                return False
        return True

    def _init_mapping(self):
        objects_path = os.path.join(
            get_package_share_directory("opossum_bringup"), "config", str(self.year), f"{self.board_config}.yaml"
        )

        data = yaml.safe_load(open(objects_path, "r"))
        self.pliers = {}
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
                
            rot = obj['shape'] == 'rot'

            for element in stacks[type_obj].values():
                x = obj['x'] + element['x'] * cos_ - element['y'] * sin_
                y = obj['y'] + element['x'] * sin_ + element['y'] * cos_
                t = obj['t'] + element['t']
                self.haz_crates[crates_count] = HazCrate(crates_count, x, y, t, rot)
                crates_count += 1

        for id, zone in data['zones'].items():
            self.zones[id] = ZoneRelease(id, zone['x'], zone['y'], zone['size'])
        
        self.final_zone_mapping = data['final_zone']

        # Process objects with cam
        self.rect = (0.45, 2.55, 0.45, 1.1) if self.board_config == "objects" else (0.45, 0.55, 0.45, 1.1)

    def _init_parameters(self) -> None:
        """Initialize the parameters of the node."""
        self.declare_parameters(
            namespace="",
            parameters=[
                ("board_config", "objects"),
                ("year", 2026),
                ("boundaries", [0.0, 3.0, 0.0, 2.0]),
            ],
        )

        self.boundaries = (
            self.get_parameter("boundaries")
            .get_parameter_value()
            .double_array_value
        )
        self.year = (
            self.get_parameter("year")
            .get_parameter_value()
            .integer_value
        )
        self.board_config = (
            self.get_parameter("board_config")
            .get_parameter_value()
            .string_value
        )

    def _init_timers(self):
        """Initialize the timers of the node."""
        self.timer_match = self.create_timer(
            100.0,
            self.timer_match_callback,
            callback_group=self.cb_group
        )
        self.timer_backstage = self.create_timer(
            92.0,
            self.timer_backstage_callback,
            callback_group=self.cb_group
        )
        self.pub_timer = self.create_timer(0.2, self.publish_board_state,callback_group=self.cb_group)

    def _reset_move_timer(self):
        """Reset the move timer."""
        if self.move_timer is not None:
            self.move_timer.cancel()
            self.move_timer = None

    def _init_publishers(self):
        """Initialize the publishers of the node."""
        self.pub_command = self.create_publisher(
            String,
            "command",
            10
        )

        self.pub_score = self.create_publisher(
            Int32,
            "score",
            10
        )

        self.pub_au = self.create_publisher(
            Bool,
            "au",
            10
        )

        self.pub_end_of_match = self.create_publisher(
            Bool,
            "end_of_match",
            10
        )

        self.pub_board_state = self.create_publisher(
            String,
            "board_state_updates",
            10
        )

    def _init_subscribers(self):
        """Initialize the subscribers of the node."""
        self.subscription = self.create_subscription(
            ParameterEvent,
            "/parameter_events",
            self.parameter_event_callback,
            10,
            callback_group=self.cb_group
        )

        self.sub_feedback = self.create_subscription(
            String,
            "feedback_command",
            self.feedback_callback,
            10,
            callback_group=self.cb_group
        )

        self.lidar_loc_sub = self.create_subscription(
            LidarLoc,
            "position_out",
            self.lidar_loc_callback,
            10,
            callback_group=self.cb_group
        )

        self.color_sub = self.create_subscription(
            String,
            "init_team_color",
            self.color_callback,
            10,
            callback_group=self.cb_group
        )

        self.aruco_sub = self.create_subscription(
            VisionDataFrame,
            "aruco_loc",
            self.aruco_callback,
            10,
            callback_group=self.cb_group
        )

        self.robot_data_sub = self.create_subscription(
            RobotData,
            "robot_data",
            self.robot_data_callback,
            10,
            callback_group=self.cb_group
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
        # --- THE FIX: Add 'zones' to the initialization dictionary ---
        full_state = {'crates': [], 'pliers': [], 'zones': []}

        for cid, crate in self.haz_crates.items():
            full_state['crates'].append({
                'id': cid, 'x': round(crate.x, 3), 'y': round(crate.y, 3),
                'theta': round(crate.theta, 3), 'state': crate.state, 'color': crate.color, 
                'reward': crate.pick_reward,
                'path': crate.best_pick_path  
            })

        # Dump all pliers
        for pid, plier in self.pliers.items():
            full_state['pliers'].append({
                'id': pid, 'x': round(plier.x, 3), 'y': round(plier.y, 3),
                'theta': round(plier.theta, 3), 'state': plier.state,
                'is_running': plier.is_running  # <-- THE FIX: Added to match the publisher!
            })

        # --- THE FIX: Dump all zones ---
        if hasattr(self, 'zones'):
            for zid, zone in self.zones.items():
                full_state['zones'].append({
                    'id': zid, 
                    'x': round(zone.x, 3), 
                    'y': round(zone.y, 3),
                    'reward': zone.release_reward,
                    'path': zone.best_release_path
                })

        # --- NEW: Add the Morbidity data for initialization ---
        if hasattr(self, 'morbidity_data'):
            full_state['morbidity'] = self.morbidity_data

        # Return it directly in the service response string
        response.success = True
        response.message = json.dumps(full_state)
        
        # --- THE FIX: Add 'zones' to the reset dictionary so the delta publisher works correctly ---
        self.last_sent_state = {'crates': {}, 'pliers': {}, 'zones': {}}
        
        self.get_logger().info("Full board state sent to map via service.")
        return response

    def publish_board_state(self):
        """Publish only the crates, pliers, and zones that have changed."""
        # Make sure we have the required attributes
        if not hasattr(self, 'haz_crates') or not hasattr(self, 'pliers') or not hasattr(self, 'zones'):
            return

        # --- THE FIX: Add 'zones' to the updates dictionary ---
        updates = {'crates': [], 'pliers': [], 'zones': [], 'morbidity': []}

        # Ensure 'zones' exists in the last_sent_state dictionary
        if 'zones' not in self.last_sent_state:
            self.last_sent_state['zones'] = {}

        # 1. Check Crates for changes
        for cid, crate in self.haz_crates.items():
            current_state = (round(crate.x, 3), round(crate.y, 3), round(crate.theta, 3), 
                             crate.state, crate.color, round(crate.pick_reward, 2))
            last_state = self.last_sent_state['crates'].get(cid)
            
            if current_state != last_state:
                updates['crates'].append({
                    'id': cid, 'x': current_state[0], 'y': current_state[1],
                    'theta': current_state[2], 'state': current_state[3], 'color': current_state[4], 
                    'reward': crate.pick_reward,
                    'path': crate.best_pick_path
                })
                self.last_sent_state['crates'][cid] = current_state

        # 2. Check Pliers for changes
        for pid, plier in self.pliers.items():
            current_state = (round(plier.x, 3), round(plier.y, 3), round(plier.theta, 3), plier.state, plier.is_running)
            last_state = self.last_sent_state['pliers'].get(pid)
            
            if current_state != last_state:
                updates['pliers'].append({
                    'id': pid, 'x': current_state[0], 'y': current_state[1],
                    'theta': current_state[2], 'state': current_state[3],
                    'is_running': plier.is_running 
                })
                self.last_sent_state['pliers'][pid] = current_state

        # 3. --- NEW: Check Zones for changes ---
        for zid, zone in self.zones.items():
            # Track changes based on reward (if reward changes, the path likely changed too)
            # Assuming zone has x, y attributes based on your GUI implementation
            current_state = (round(zone.x, 3), round(zone.y, 3), round(zone.release_reward, 2))
            last_state = self.last_sent_state['zones'].get(zid)
            
            if current_state != last_state:
                updates['zones'].append({
                    'id': zid, 
                    'x': current_state[0], 
                    'y': current_state[1],
                    'reward': zone.release_reward,
                    'path': zone.best_release_path
                })
                self.last_sent_state['zones'][zid] = current_state
        
        # 4. --- NEW: Check Morbidity Map for changes ---
        if hasattr(self, 'morbidity_data'):
            # We compare the current morbidity data to the last one sent
            # Using a simple comparison of the dict (or a hash if it gets too big)
            if self.morbidity_data != self.last_sent_state.get('morbidity'):
                updates['morbidity'] = self.morbidity_data
                self.last_sent_state['morbidity'] = self.morbidity_data.copy()

        # 4. Only publish if there is actually something new in ANY of the lists!
        if updates['crates'] or updates['pliers'] or updates['zones'] or updates['morbidity']:
            msg = String()
            msg.data = json.dumps(updates)
            self.pub_board_state.publish(msg)

    # =========================================================================
    # UNIFIED CAMERA CALLBACK (HUNGARIAN TRACKING WITH ARUCO ID)
    # =========================================================================
    def aruco_callback(self, msg: VisionDataFrame):
        """Continuously save the latest camera frame without processing it."""
        if msg.id == 1 or msg.id == 2:
            self.latest_camera_msg[msg.id] = msg
            self.last_camera_timestamp[msg.id] = time.time()

    def _extract_color_from_id(self, aruco_id: int) -> int:
        """Map ArUco ID to internal color code."""
        if aruco_id == 47:
            return 0  # Yellow
        elif aruco_id == 36:
            return 1  # Blue
        elif aruco_id == 41:
            return 2  # Rot (Red)
        return -1 # Unknown

    def stare_and_update(self, crate_dict):
        """Stop, let the camera settle, and process the latest frame in World Coordinates."""
        if self.stop:
            return

        if getattr(self, 'robot_pos', None) is None:
            self.get_logger().warn("Stare failed: Robot position unknown.")
            return

        self.get_logger().info("Staring... waiting for camera to settle.")
        
        # 1. Wait for physical motion blur to clear
        time.sleep(1.0)

        for key, msg in self.latest_camera_msg.items():
            if time.time() - self.last_camera_timestamp[key] > 0.2:
                continue

            # 2. Grab the latest message in the buffer
            if msg is None or not msg.object:
                self.get_logger().info(f"Stare complete: No objects currently visible for camera {key}.")
                continue

            camera_detections = msg.object
            current_time = time.time()

            # =====================================================================
            # 3. TRANSFORM DETECTIONS: ROBOT FRAME -> WORLD FRAME
            # =====================================================================
            world_detections = []
            cos_t = math.cos(self.robot_pos.t)
            sin_t = math.sin(self.robot_pos.t)

            for det in camera_detections:
                if det.x ** 2 + det.y ** 2 < 0.05 and det.z > 0.17:
                    continue
                if det.x ** 2 + det.y ** 2 > 0.6 ** 2:
                    continue
                world_det = SimpleNamespace()
                world_det.id = det.id
                world_det.x = self.robot_pos.x + (det.x * cos_t - det.y * sin_t)
                world_det.y = self.robot_pos.y + (det.x * sin_t + det.y * cos_t)
                world_det.theta = self.robot_pos.t + det.theta
                world_detections.append(world_det)

            # =====================================================================
            # 4. HUNGARIAN TRACKING LOGIC (Using World Detections)
            # =====================================================================

            crate_ids = list(crate_dict.keys())
            num_crates = len(crate_ids)
            num_detections = len(world_detections)
            
            cost_matrix = np.zeros((num_crates, num_detections))

            for i, cid in enumerate(crate_ids):
                crate = crate_dict[cid]
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
                    matched_crate = crate_dict[cid]
                    det = world_detections[det_idx]

                    matched_crate.x = det.x
                    matched_crate.y = det.y
                    matched_crate.theta = det.theta
                    matched_crate.last_seen = current_time
                    matched_crate.color = self._extract_color_from_id(det.id)

                    # if matched_crate.color in [-1, 2]:

                    matched_detection_indices.add(det_idx)

            for j, det in enumerate(world_detections):
                if j not in matched_detection_indices:
                    new_id = max(crate_dict.keys()) + 1
                    color_val = self._extract_color_from_id(det.id)
                    new_crate = HazCrate(new_id, det.x, det.y, det.theta, rot=(color_val == 2))
                    new_crate.color = color_val
                    new_crate.last_seen = current_time
                    crate_dict[new_id] = new_crate
                    self.get_logger().info(f"Tracking: Discovered new crate! Assigned internal ID: {new_id} at X:{det.x:.2f} Y:{det.y:.2f}")

        # =====================================================================
        # 5. GHOST CLEANUP (FOV & Range Verification)
        # =====================================================================
        # ghost_ids = []
        # fov_half = math.radians(65.0 / 2.0) # 65 degrees FOV divided by 2
        # max_range = 0.80 # 80cm limit

        # for cid, crate in self.haz_crates.items():
        #     if crate.last_seen == current_time or crate.state != -1:
        #         continue
                
        #     # Distance to crate
        #     dist = math.hypot(crate.x - self.robot_pos.x, crate.y - self.robot_pos.y)
            
        #     # If it is close enough to see...
        #     if dist <= max_range:
        #         angle_to_crate = math.atan2(crate.y - self.robot_pos.y, crate.x - self.robot_pos.x)
        #         is_in_view = False
                
        #         # Check if it falls inside any of our 3 camera cones!
        #         for cam_angle in self.camera_angles:
        #             cam_global_angle = self.robot_pos.t + cam_angle
        #             diff = self.angular_distance(angle_to_crate, cam_global_angle)
                    
        #             if diff <= fov_half:
        #                 is_in_view = True
        #                 break
                        
        #         if is_in_view:
        #             # Should be perfectly visible, but wasn't detected!
        #             ghost_ids.append(cid)

        # self.get_logger().info("Finished staring go back to match.")

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
        self.final_zone = self.final_zone_mapping[self.color]
        if self.color == 0:
            self.steal_poses = [[0.47, 1.1], [2.53, 1.1], [2.53, 0.45], [0.47, 0.45]]
        if self.color == 1:
            self.steal_poses = [[2.53, 1.1], [0.47, 1.1], [0.47, 0.45], [2.53, 0.45]]


    def feedback_callback(self, msg):
        """Receive the data from Zynq."""
        if msg.data.startswith("LEASH") and self.ready and not self.is_started:
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
                self.pub_au.publish(Bool(data=False))

        elif msg.data.startswith("BLUESWITCH"):
            self.get_logger().info("Reload Ros")
            os.system('systemctl --user restart launch.service')

        elif msg.data.strip() == "Pos,done":
            self.motion_done = True
            self.motion_done_event.set()

        elif msg.data.startswith("PINCEFEEDBACK"):
            data = msg.data.split()[1:]
            cmd_id = int(data[0])
            if cmd_id == 10:
                for plier in self.pliers.values():
                    if plier.is_running:
                        plier.is_running = False
                        # crate = self.haz_crates[plier.state]
                        # crate.state = -1
                        # plier.state = -1
            else:
                # Map the two success bits and the two plier objects
                # v0 = data[2], v1 = data[3]
                results = [
                    (int(data[2]), self.pliers[cmd_id * 2], "first"),
                    (int(data[3]), self.pliers[cmd_id * 2 + 1], "second")
                ]

                for success_bit, plier, label in results:
                    # CRITICAL: Only check success IF the plier was actually told to move
                    if plier.is_running:
                        plier.is_running = False
                        
                        # If firmware reports 0 while we expected a success
                        if not success_bit and plier.state != -1:
                            crate = self.haz_crates[plier.state]
                            self.get_logger().warn(f"The {label} plier of ID {cmd_id} failed.")
                            
                            # Reset tracking
                            crate.state = -1
                            crate.attempts += 1
                            plier.state = -1

            # Global check to release the event loop
            if not any(pl.is_running for pl in self.pliers.values()):
                self.pliers_event.set()
    
    def continuous_planner_callback(self):
        """Background thread: Continuously scores every crate and zone on the map."""
        if self.stop or not hasattr(self, 'haz_crates') or getattr(self, 'robot_pos', None) is None:
            return

        max_pliers = 2
        with self.data_lock:
            self.update_morbidity_map()

            # Reset rewards
            for crate in self.haz_crates.values():
                crate.pick_reward = float('-inf')
            
            for zone in self.zones.values():
                zone.release_reward = float('-inf')

            # Compute the amount of pliers active
            pliers_active = 0
            for side in range(4):
                if any(self.pliers[pid].state != -1 for pid in range(side * 4, (side + 1) * 4)):
                    pliers_active += 1
            
            # Check if more than 2
            if pliers_active <= max_pliers:
                self.update_pick_reward()

            if  not all(pl.state == -1 for pl in self.pliers.values()):
                self.update_release_reward()

    def update_pick_reward(self):
        current_stacks = self.generate_stacks(self.haz_crates)
        approach_distance = 0.26

        for stack_ids in current_stacks:
            group_crates = [self.haz_crates[cid] for cid in stack_ids if cid in self.haz_crates]
            if not group_crates:
                continue

            # Security Filter: If any crate in stack is safe, skip the whole stack
            skip_stack = False
            for crate in group_crates:
                if (self.get_current_zone(crate.x, crate.y, crate.theta) is not None and crate.color in (-1, self.color, 2)) or self.in_final_zone(crate.x, crate.y, crate.theta):
                    skip_stack = True
                    break
            
            if skip_stack:
                continue

            # Consolidation math
            mean_x = np.mean([c.x for c in group_crates])
            mean_y = np.mean([c.y for c in group_crates])
            mean_theta = np.arctan2(np.sum([np.sin(c.theta) for c in group_crates]), 
                                    np.sum([np.cos(c.theta) for c in group_crates]))

            # Calculate Entry Points
            entries = [
                (mean_x - approach_distance * np.cos(mean_theta), mean_y - approach_distance * np.sin(mean_theta), False),
                (mean_x + approach_distance * np.cos(mean_theta), mean_y + approach_distance * np.sin(mean_theta), True)
            ]

            best_dist = float('inf')
            best_path = []
            best_inv = False

            primary_id = stack_ids[0]

            for ex, ey, is_inv in entries:
                if not (self.boundaries[0] + self.robot_radius <= ex <= self.boundaries[1] - self.robot_radius and self.boundaries[2] + self.robot_radius <= ey <= self.boundaries[3] - self.robot_radius):
                    continue
                    
                target_pos = (ex, ey)
                
                # Check Path
                path, dist = self.get_best_path(target_pos, target_crate_id=primary_id, allow_critical=False)

                if dist < best_dist:
                    best_dist = dist
                    best_path = path
                    best_inv = is_inv

            # Write the final score into ALL crates in this stack
            if best_dist != float('inf'):
                reward = self.compute_pick_penality(mean_x, mean_y)
                for cid in stack_ids:
                    if cid in self.haz_crates:
                        crate = self.haz_crates[cid]
                        crate.pick_reward = reward + (-15) * crate.attempts
                        crate.best_pick_path = best_path
                        crate.use_inverted = best_inv
                        crate.is_part_of_stack = stack_ids

    def update_release_reward(self):
        distance = 0.26
        for z_id, zone in list(self.zones.items()):
            # Check if occupied
            if len(self.get_all_points_in_zone(z_id, self.haz_crates)) > 0:
                continue 
            
            x = zone.x
            y = zone.y
            av_poses = [
                [x + distance, y, 3.14],
                [x - distance, y, 0.0],
                [x, y + distance, 4.71],
                [x, y - distance, 1.57],
            ]

            best_dist = float('inf')
            best_path = []
            best_pos = None

            for pos in av_poses:
                if not (self.boundaries[0] + self.robot_radius < pos[0] < self.boundaries[1] - self.robot_radius and 
                        self.boundaries[2] + self.robot_radius < pos[1] < self.boundaries[3] - self.robot_radius): 
                    continue
                    
                target_pos = (pos[0], pos[1])
                
                path, dist = self.get_best_path(target_pos, allow_critical=False)
                        
                if dist < best_dist:
                    best_dist = dist
                    best_path = path
                    best_pos = pos

            # Write the final score into the zone
            if best_dist != float('inf'):
                reward = self.compute_release_penality(best_pos[0], best_pos[1], best_dist)
                zone.release_reward = reward
                zone.best_release_path = best_path
                zone.best_release_pos = best_pos

    def update_morbidity_map(self):
        """Call this inside continuous_planner to refresh the 'danger' data."""
        with self.data_lock:
            self.morbidity_data = {
                "hard_zones": [],      # Static boundaries
                "safety_zones": [],    # Expanded boundaries (0.35m)
                "crate_bubbles": []    # Dynamic circles around crates
            }

            # 1. Store the Static Forbidden Zone
            # Original: (0.6, 1.55) to (2.4, 2.0)
            self.morbidity_data["hard_zones"].append({
                "x": 0.6, "y": 1.55, "w": 1.8, "h": 0.45
            })

            # 2. Store the Expanded 'Minkowski' Zone (where robot center can't go)
            self.morbidity_data["safety_zones"].append({
                "x": self.f_zone_x_min, 
                "y": self.f_zone_y_min, 
                "w": self.f_zone_x_max - self.f_zone_x_min, 
                "h": self.f_zone_y_max - self.f_zone_y_min
            })

            # 3. Store Crate Bubbles (Dynamic)s
            for cid, crate in self.haz_crates.items():
                if crate.state == -1: # Only floor crates
                    self.morbidity_data["crate_bubbles"].append({
                        "id": cid,
                        "x": crate.x,
                        "y": crate.y,
                        "radius": 0.22 # Ideal Margin
                    })

    def robot_data_callback(self, msg: RobotData):
        """Receive the Robot Data from Zynq."""
        with self.data_lock:
            self.robot_pos = Position(x=msg.x, y=msg.y, t=msg.theta)
            self.robot_speed = Position(x=msg.vlin, y=msg.vdir, t=msg.vt)

        if not self.motion_done:
            # Update motion state
            self.update_arrival_status()
            is_stopped = abs(self.robot_speed.x) < 0.02 and abs(self.robot_speed.t) < 0.02

            # Si on est proche de la cible ET à l'arrêt, on débloque le script.
            if self.is_robot_arrived and is_stopped:
                self.motion_done = True
                self.motion_done_event.set() # C'est ça qui débloque wait_for_motion

    def timer_match_callback(self):
        """Timer callback for match time."""
        self.pub_end_of_match.publish(Bool(data=True))
        self.match_finished = True
        self.get_logger().warn("Match time exceeded")
        self.stop_script()

    def timer_backstage_callback(self):
        self.backstage_sequence = True
        self.get_logger().warn("Abort, go back home")
        self.send_raw("VMAX 1.5")
        self.get_logger().info("Cannot find release zone. Going to final zone.")
        
        if self.color == 0:
            e_zone = (0.45, 1.2)
        elif self.color == 1:
            e_zone = (1.55, 1.2)
        else:
            e_zone = (0.45, 1.1)
        path, _ = self.get_best_path(e_zone, allow_critical=True)
        self.navigate_path(path, 0.0)

        path, _ = self.get_best_path((self.final_zone["x"], self.final_zone["y"]), allow_critical=True)
        self.navigate_path(path, 0.0)
            
        with self.data_lock:
            self.send_raw("PINCE 10 0 0")
            id_active_pliers_all = {}
            for plier in self.pliers.values():
                if plier.state == -1:
                    continue
                crate = self.haz_crates[plier.state]
                plier.state = -1
                plier.is_running = True
                crate.state = -1
                self.update_crate_pos(plier, crate)

        self.send_plier_cmd(id_active_pliers_all, self.haz_crates)
        self.wait_for_plier()

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
            self.send_raw("FREE")
            self.end_match_event.set()
            time.sleep(0.1)
            self.send_raw("FREE")
            self.script_instance = None
            self.script_thread = None

    def lidar_loc_callback(self, msg: LidarLoc):
        """Receive Lidar location."""
        self.lidar_pos = Position(
            x=msg.robot_position.x,
            y=msg.robot_position.y,
            t=msg.robot_position.z
        )

        list_enn = msg.other_robot_position
        if len(list_enn) == 0:
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
            # self.get_logger().info(f'Move To {pos.x}, {pos.y}, {pos.t}')
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
            self.pliers_event.wait(timeout=10.0)

    def send_plier_cmd(self, ids: dict, crate_dict: dict):
        if self.stop or not ids:
            return

        self.pliers_event.clear()
        action_map = {"pick": 1, "drop": 2, "rev_drop": 3}
        
        # Special modes for mixed pairs: (Action 0, Action 1) -> Mode ID
        pair_modes = {
            ("drop", "rev_drop"): 4,
            ("rev_drop", "drop"): 5
        }

        sorted_ids = sorted(ids.keys())
        processed = set()

        for i in range(len(sorted_ids)):
            curr_id = sorted_ids[i]
            if curr_id in processed:
                continue

            cmd_id = curr_id // 2
            pair_id = curr_id + 1
            
            # Determine if we can run as a PAIR (Mode X 2)
            if curr_id % 2 == 0 and pair_id in ids:
                act1, act2 = ids[curr_id][0], ids[pair_id][0]
                
                # Find the mode: either they are the same, or they match a special mixed mode
                mode = None
                if act1 == act2:
                    mode = action_map.get(act1)
                else:
                    mode = pair_modes.get((act1, act2))

                if mode:
                    # Execute as Pair
                    self.pub_command.publish(String(data=f"PINCE {cmd_id} {mode} 2"))
                    self._update_plier_logic(curr_id, ids[curr_id], crate_dict)
                    self._update_plier_logic(pair_id, ids[pair_id], crate_dict)
                    processed.update([curr_id, pair_id])
                    continue

            # Execute Individually
            act, crate_id = ids[curr_id]
            mode = action_map.get(act, 1)
            side = curr_id % 2
            self.pub_command.publish(String(data=f"PINCE {cmd_id} {mode} {side}"))
            self._update_plier_logic(curr_id, ids[curr_id], crate_dict)
            processed.add(curr_id)

    def _update_plier_logic(self, plier_id, action_info, crate_dict):
        """Helper to sync internal state after a command is sent."""
        action, crate_id = action_info
        plier = self.pliers[plier_id]
        crate = crate_dict[crate_id]
        
        plier.is_running = True

        if action == "pick":
            plier.state = crate.id
            crate.state = plier_id
        else:
            # Handling drop / rev_drop
            plier.state = -1
            crate.state = -1
            if action == "rev_drop":
                crate.color = 1 if crate.color == 0 else 0
            
            self.update_crate_pos(plier, crate)

    def update_crate_pos(self, plier, crate):
        """Update the position of the crate in the world reference."""
        # 1. Grab the robot's current global pose
        rx = self.robot_pos.x
        ry = self.robot_pos.y
        rt = self.robot_pos.t
        
        # 2. Pre-compute sin and cos for efficiency
        cos_t = math.cos(rt)
        sin_t = math.sin(rt)
        
        # 3. Apply the 2D rotation and translation to the plier's local coordinates
        crate.x = rx + (plier.x * cos_t) - (plier.y * sin_t)
        crate.y = ry + (plier.x * sin_t) + (plier.y * cos_t)
        
        # 4. The global angle is simply the robot's angle plus the actuator's angle
        crate.theta = rt + plier.theta

    def send_raw(self, raw_command):
        """Send raw commands."""
        if not self.stop:
            self.pub_command.publish(String(data=raw_command))
            time.sleep(0.1)

    def kalman(self, kalman: bool):
        """Compute the kalman action."""
        if not self.stop:
            self.pub_command.publish(String(data=f"ENKALMAN {int(kalman)}"))
            time.sleep(0.1)

    def generate_stacks(self, crate_dict):
        """
        Calculates stacks using Geometric Adjacency and Connected Components.
        Rules: Crates must be parallel, aligned lengthwise, side-by-side (~0.05m apart), max 4 per stack.
        """
        # --- Strict Geometric Tolerances ---
        WIDTH = 0.05  # Ideal distance between centers when side-by-side
        ANGLE_TOLERANCE = math.radians(15)  # 15 degrees max rotation difference
        LONG_TOLERANCE = 0.03  # Allow up to 3cm of sliding/misalignment lengthwise
        LAT_TOLERANCE = 0.02   # Allow the lateral gap to be between 3cm and 7cm
        
        with self.data_lock:
            # 1. Grab only free crates
            available_crates = [c for c in crate_dict.values() if c.state == -1]
            
            # 2. Build an Adjacency Graph (Dictionary of connections)
            adjacency = {c.id: [] for c in available_crates}
            
            for i in range(len(available_crates)):
                c1 = available_crates[i]
                for j in range(i + 1, len(available_crates)):
                    c2 = available_crates[j]
                    
                    # Rule A: Angles must be parallel. 
                    # math.remainder(diff, pi) handles crates that are flipped 180 degrees!
                    angle_diff = math.remainder(c1.theta - c2.theta, math.pi)
                    if abs(angle_diff) > ANGLE_TOLERANCE:
                        continue
                        
                    # Rule B: Calculate the relative distance
                    dx = c2.x - c1.x
                    dy = c2.y - c1.y
                    
                    # Project that distance onto Crate 1's local axes
                    cos_t = math.cos(c1.theta)
                    sin_t = math.sin(c1.theta)
                    
                    long_dist = abs(dx * cos_t + dy * sin_t)
                    lat_dist = abs(-dx * sin_t + dy * cos_t)
                    
                    # Rule C: They must be perfectly side-by-side
                    # long_dist ~ 0 (flush ends), lat_dist ~ 0.05 (touching sides)
                    if long_dist <= LONG_TOLERANCE and abs(lat_dist - WIDTH) <= LAT_TOLERANCE:
                        # They are geometrically locked! Connect them in the graph.
                        adjacency[c1.id].append(c2.id)
                        adjacency[c2.id].append(c1.id)
                        
            # 3. Find Connected Components (The Stacks)
            visited = set()
            stack_ids_only = []
            
            for crate in available_crates:
                if crate.id in visited:
                    continue
    
                # Run a quick BFS to collect all crates connected in this specific row
                component = []
                queue = [crate.id]
                
                while queue:
                    curr = queue.pop(0)
                    if curr in visited:
                        continue

                    visited.add(curr)
                    component.append(curr)
                    # Add all valid neighbors to the queue
                    queue.extend([neighbor for neighbor in adjacency[curr] if neighbor not in visited])
                
                # 4. Enforce the Max 4 Rule
                # If we found 5 crates in a row, slice it into [4] and [1]
                for i in range(0, len(component), 4):
                    stack_ids_only.append(component[i:i+4])
        return stack_ids_only

    def centering(self):
        pass
    
    def test_script0(self):
        for _ in range(10):
            self.send_raw("PINCE 0 1 2")
            self.send_raw("PINCE 1 1 2")
            time.sleep(5)
            self.send_raw("PINCE 0 2 2")
            self.send_raw("PINCE 1 2 2")
            time.sleep(5)
    
    def test_script1(self):
        for _ in range(10):
            self.send_raw("PINCE 2 1 2")
            self.send_raw("PINCE 3 1 2")
            self.send_raw("PINCE 4 1 2")
            self.send_raw("PINCE 5 1 2")
            time.sleep(5)
            self.send_raw("PINCE 2 2 2")
            self.send_raw("PINCE 3 2 2")
            self.send_raw("PINCE 4 1 2")
            self.send_raw("PINCE 5 1 2")
            time.sleep(5)

    def test_script2(self):
        for _ in range(10):
            self.send_raw("PINCE 4 1 2")
            self.send_raw("PINCE 5 1 2")
            time.sleep(5)
            self.send_raw("PINCE 4 2 2")
            self.send_raw("PINCE 5 2 2")
            time.sleep(5)

    def test_script3(self):
        for _ in range(10):
            self.send_raw("PINCE 6 1 2")
            self.send_raw("PINCE 7 1 2")
            time.sleep(5)
            self.send_raw("PINCE 6 2 2")
            self.send_raw("PINCE 7 2 2")
            time.sleep(5)

    def smart_moves(self):
        # self.send_raw("VMAX 1.5")
        # self.send_raw("VTMAX 1.5")
        id_steal = 0

        while not self.backstage_sequence:
            self.send_raw("VMAX 1.5")
            self.send_raw("VTMAX 2.0")
            self.continuous_planner_callback()
            action = None
            
            # =================================================================
            # 1. INSTANTLY CHOOSE THE BEST ACTION FROM CACHED SCORES
            # =================================================================
            with self.data_lock:
                best_zone = max(self.zones.values(), key=lambda z: z.release_reward, default=None)
                best_crate = max(self.haz_crates.values(), key=lambda c: c.pick_reward, default=None)

                if best_crate and best_crate.pick_reward > float('-inf'):
                    pick_crate_ids = best_crate.is_part_of_stack
                    pick_path = best_crate.best_pick_path
                    is_inv = best_crate.use_inverted
                    action = "PICK"
                
                elif best_zone and best_zone.release_reward > float('-inf'):
                    rel_path = best_zone.best_release_path
                    best_release_pos = best_zone.best_release_pos
                    action = "RELEASE"     
                
                else:
                    action = "EXPLORE"

            # =================================================================
            # 2. EXECUTE RELEASE
            # =================================================================
            if action == "RELEASE":
                self.get_logger().info(f"Executing RELEASE at {best_release_pos}")
                with self.data_lock:
                    id_side = self.get_best_side_pliers_release(best_release_pos[2])
                    pliers_theta = self.pliers[id_side * 4].theta if id_side is not None else 0.0
                    
                target_angle = best_release_pos[2] - pliers_theta
                
                self.navigate_path(rel_path, target_angle)
                if self.backstage_sequence:
                    break
                self.stare_and_update(self.haz_crates)

                with self.data_lock:
                    id_active_pliers = {}
                    for pid, plier in self.pliers.items():
                        if plier.state == -1 or pid // 4 != id_side:
                            continue
                        
                        crate = self.haz_crates[plier.state]
                        if crate.color in (-1, 2) or crate.color == self.color:
                            id_active_pliers[pid] = ["drop", plier.state]
                        else:
                            id_active_pliers[pid] = ["rev_drop", plier.state]
                if self.backstage_sequence:
                    break
                self.send_plier_cmd(id_active_pliers, self.haz_crates)
                self.wait_for_plier()

                with self.data_lock:
                    for pl_id in id_active_pliers:
                        self.pliers[pl_id].state = -1
                continue

            # =================================================================
            # 4. EXECUTE PICK
            # =================================================================
            elif action == "PICK":
                with self.data_lock:
                    pliers_config = self.get_best_side_pliers(len(pick_crate_ids))

                if not pliers_config:
                    self.get_logger().warn("Target found but no pliers available. Switching to Release.")
                    continue
                
                selected_pliers_ids = pliers_config[0]

                with self.data_lock:
                    target_pos = self.get_mean_pose([self.haz_crates[pid] for pid in pick_crate_ids])
                    pliers_pos = self.get_mean_pose([self.pliers[pid] for pid in selected_pliers_ids])
                
                target_angle = target_pos.t + (np.pi if is_inv else 0.0)
                final_robot_theta = target_angle - pliers_pos.t

                x_offset, y_offset = self.rotate_point(pliers_pos.x, pliers_pos.y, final_robot_theta)
                
                final_pos = Position(
                    target_pos.x - x_offset,
                    target_pos.y - y_offset,
                    final_robot_theta
                )

                entry_point = pick_path[-1] if len(pick_path) > 0 else (self.robot_pos.x, self.robot_pos.y)
                
                if len(pick_path) > 1:
                    for i, wp in enumerate(pick_path[1:]):
                        if self.backstage_sequence:
                            break
                        self.move_to(Position(wp[0], wp[1], final_robot_theta))
                        self.wait_for_motion()
                if self.backstage_sequence:
                    break
                self.stare_and_update(self.haz_crates)
                self.centering()

                if self.backstage_sequence:
                    break
                # self.send_raw("ENKALMAN 0 0")
                # self.move_to(Position(entry_point[0], entry_point[1], final_robot_theta))
                # self.get_logger().info(f'Entry Point {entry_point[0]}, {entry_point[1]}, {final_robot_theta}')
                # self.wait_for_motion()

                if self.backstage_sequence:
                    break

                self.move_to(final_pos)
                # self.get_logger().info(f'Final Pose {final_pos.x}, {final_pos.y}, {final_robot_theta}')
                self.wait_for_motion()
                # self.send_raw("ENKALMAN 1 1")

                with self.data_lock:
                    robot_x, robot_y, robot_t = final_pos.x, final_pos.y, final_pos.t
                    pliers_in_map = {}
                    for pl_id in selected_pliers_ids:
                        p_local = self.pliers[pl_id]
                        rx, ry = self.rotate_point(p_local.x, p_local.y, robot_t)
                        pliers_in_map[pl_id] = (robot_x + rx, robot_y + ry)

                    remaining_targets = [tid for tid in pick_crate_ids if tid in self.haz_crates]
                    
                    if not remaining_targets:
                        self.get_logger().warn("All targets vanished during approach! Aborting.")
                        continue 

                    plier_list = list(pliers_in_map.keys())
                    target_list = list(remaining_targets)
                    cost_matrix = np.zeros((len(plier_list), len(target_list)))
                    
                    for i, pl_id in enumerate(plier_list):
                        px, py = pliers_in_map[pl_id]
                        for j, obj_id in enumerate(target_list):
                            cx = self.haz_crates[obj_id].x
                            cy = self.haz_crates[obj_id].y
                            cost_matrix[i, j] = (cx - px)**2 + (cy - py)**2

                    row_ind, col_ind = linear_sum_assignment(cost_matrix)

                    dict_sel_pliers = {}
                    for i, j in zip(row_ind, col_ind):
                        pl_id = plier_list[i]
                        best_obj_id = target_list[j]
                        dict_sel_pliers[pl_id] = ["pick", best_obj_id]

                if self.backstage_sequence:
                    break
                self.send_plier_cmd(dict_sel_pliers, self.haz_crates)
                self.wait_for_plier()
                continue

            # =================================================================
            # 5. EXECUTE EXPLORE / STEAL
            # =================================================================
            elif action == "EXPLORE":
                self.get_logger().info("Nothing to do: Exploring / Stealing...")
                pos = self.steal_poses[id_steal % len(self.steal_poses)]

                if self.backstage_sequence:
                    break
                
                if self.is_point_safe(pos[0], pos[1]):
                    path, _ = self.get_best_path((pos[0], pos[1]), allow_critical=True)
                    self.navigate_path(path, (id_steal * 2.3998) % (2 * np.pi))
                    self.stare_and_update(self.haz_crates)
                    
                id_steal += 1

    def get_mean_pose(self, objects):
        """Calcule la position moyenne (x, y, theta_circulaire) d'une liste d'objets."""
        mx = np.mean([obj.x for obj in objects])
        my = np.mean([obj.y for obj in objects])
        m_sin = np.mean([np.sin(obj.theta) for obj in objects])
        m_cos = np.mean([np.cos(obj.theta) for obj in objects])
        return Position(mx, my, np.arctan2(m_sin, m_cos))

    def get_current_zone(self, x, y, theta):
        """
        Checks if ANY part of a rotated crate (0.15m x 0.05m) is inside the zone.
        Uses the Separating Axis Theorem (SAT) for OBB-AABB collision.
        """
        # Crate dimensions
        hl = 0.15 / 2.0  # Half-length
        hw = 0.05 / 2.0  # Half-width
        
        cos_t = math.cos(theta)
        sin_t = math.sin(theta)
        
        # 1. Calculate the crate's projected "radius" on the global X and Y axes
        r_cx = abs(hl * cos_t) + abs(hw * sin_t)
        r_cy = abs(hl * sin_t) + abs(hw * cos_t)
        
        for zone_id, sq in self.zones.items():
            zx = sq.x
            zy = sq.y
            hz = sq.size / 2.0  # Zone half-size
            
            # --- Axis 1 & 2: Global X and Y (The Zone's flat edges) ---
            if abs(x - zx) > (hz + r_cx):
                continue # Shadows don't overlap on X, skip to next zone!
                
            if abs(y - zy) > (hz + r_cy):
                continue # Shadows don't overlap on Y, skip to next zone!
                
            # --- Axis 3 & 4: The Crate's Local X and Y (The Crate's rotated edges) ---
            # Calculate the zone's projected "radius" onto the rotated crate's axes
            r_z_local = hz * (abs(cos_t) + abs(sin_t))
            
            # Vector from Crate center to Zone center
            dx = zx - x
            dy = zy - y
            
            # Project that vector onto the Crate's local X axis
            dx_local = abs(dx * cos_t + dy * sin_t)
            if dx_local > (hl + r_z_local):
                continue # Shadows don't overlap on local X!
                
            # Project that vector onto the Crate's local Y axis
            dy_local = abs(-dx * sin_t + dy * cos_t)
            if dy_local > (hw + r_z_local):
                continue # Shadows don't overlap on local Y!
                
            # If we survived all 4 axis checks, they are definitively colliding!
            return zone_id
            
        return None

    def in_final_zone(self, x, y, theta):
        """
        Checks if ANY part of a rotated crate (0.15m x 0.05m) is inside the zone.
        Uses the Separating Axis Theorem (SAT) for OBB-AABB collision.
        """
        # Crate dimensions
        hl = 0.15 / 2.0  # Half-length
        hw = 0.05 / 2.0  # Half-width
        
        cos_t = math.cos(theta)
        sin_t = math.sin(theta)
        
        # 1. Calculate the crate's projected "radius" on the global X and Y axes
        r_cx = abs(hl * cos_t) + abs(hw * sin_t)
        r_cy = abs(hl * sin_t) + abs(hw * cos_t)
        
        if self.final_zone is None:
            self.get_logger().info(f"Final zone not set. Skipping the check in final zone considering false.")
            return False

        zx = self.final_zone['x']
        zy = self.final_zone['y']
        hz = self.final_zone['size'] / 2.0  # Zone half-size
        
        # --- Axis 1 & 2: Global X and Y (The Zone's flat edges) ---
        if abs(x - zx) > (hz + r_cx):
            return False # Shadows don't overlap on X, skip to next zone!
            
        if abs(y - zy) > (hz + r_cy):
            return False # Shadows don't overlap on Y, skip to next zone!
            
        # --- Axis 3 & 4: The Crate's Local X and Y (The Crate's rotated edges) ---
        # Calculate the zone's projected "radius" onto the rotated crate's axes
        r_z_local = hz * (abs(cos_t) + abs(sin_t))
        
        # Vector from Crate center to Zone center
        dx = zx - x
        dy = zy - y
        
        # Project that vector onto the Crate's local X axis
        dx_local = abs(dx * cos_t + dy * sin_t)
        if dx_local > (hl + r_z_local):
            return False # Shadows don't overlap on local X!
            
        # Project that vector onto the Crate's local Y axis
        dy_local = abs(-dx * sin_t + dy * cos_t)
        if dy_local > (hw + r_z_local):
            return False # Shadows don't overlap on local Y!
            
        # If we survived all 4 axis checks, they are definitively colliding!
        return True
            
    def get_all_points_in_zone(self, id_zone, crate_dict):
        """
        Finds all haz_crates that have ANY part inside a specific square zone.
        Uses SAT for precise OBB-AABB collision. Crates are already in global coordinates.
        """
        if id_zone not in self.zones:
            return []
            
        zone = self.zones[id_zone]
        zx = zone.x
        zy = zone.y
        hz = zone.size / 2.0  # Zone half-size
        
        # --- Crate Dimensions ---
        hl = 0.15 / 2.0  # Half-length
        hw = 0.05 / 2.0  # Half-width
        
        points_inside = []
        
        # Lock the data so the camera thread doesn't change haz_crates while we iterate!
        with self.data_lock:
            for c in crate_dict.values():
                # Crates are already in global map coordinates, so no transform needed!
                cx = c.x
                cy = c.y
                ct = c.theta
                
                cos_ct = math.cos(ct)
                sin_ct = math.sin(ct)
                
                # 1. Calculate the crate's projected "radius" on global X and Y
                r_cx = abs(hl * cos_ct) + abs(hw * sin_ct)
                r_cy = abs(hl * sin_ct) + abs(hw * cos_ct)
                
                # --- SAT Axis 1 & 2: Global X and Y ---
                if abs(cx - zx) > (hz + r_cx):
                    continue # Shadows don't overlap on X
                if abs(cy - zy) > (hz + r_cy):
                    continue # Shadows don't overlap on Y
                    
                # --- SAT Axis 3 & 4: Crate's Local X and Y ---
                r_z_local = hz * (abs(cos_ct) + abs(sin_ct))
                
                dx = zx - cx
                dy = zy - cy
                
                dx_local = abs(dx * cos_ct + dy * sin_ct)
                if dx_local > (hl + r_z_local):
                    continue # Shadows don't overlap on crate's local X
                    
                dy_local = abs(-dx * sin_ct + dy * cos_ct)
                if dy_local > (hw + r_z_local):
                    continue # Shadows don't overlap on crate's local Y
                    
                # If it survives all 4 checks, the crate is definitively touching the zone!
                points_inside.append(c.id)
                
        return points_inside

    def rotate_point(self, x, y, theta):
        """Applique une rotation 2D simple."""
        cos_t, sin_t = np.cos(theta), np.sin(theta)
        rx = x * cos_t - y * sin_t
        ry = x * sin_t + y * cos_t
        return rx, ry

    def compute_release_penality(self, px, py, path_distance):
        """Calculates the score of a specific release pose using actual path distance."""
        coeff_center = 1 
        coeff_dst = -1
        coeff_enn = 0.2
        
        val_center = (1.5 - px) ** 2 + (1 - py) ** 2
        # Use the TRUE path distance, not just a straight line guess!
        val_dst = path_distance**2 
        
        if self.x_enn is not None:
            val_ennemi = (self.x_enn - px) ** 2 + (self.y_enn - py) ** 2
        else:
            val_ennemi = 0
            
        return (coeff_dst * val_dst) + (coeff_enn * val_ennemi) + (coeff_center * val_center)

    def compute_pick_penality(self, x, y):
        coeff_center = -1 # -0.005
        coeff_dst = -1
        coeff_enn = 0.2 # 0.05
        # coeef_end = -0.0001
        val_center = (1.5 - x) ** 2 + (1 - y) ** 2
        val_dst = (self.robot_pos.x - x) ** 2 + (self.robot_pos.y - y) ** 2
        if self.x_enn is not None:
            val_ennemi = (self.x_enn - x) ** 2 + (self.y_enn - y) ** 2
        else:
            val_ennemi = 0
        return coeff_dst * val_dst + coeff_enn * val_ennemi + coeff_center * val_center

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
            if max_consecutive == 4:
            # if max_consecutive >= stack_len:
                available_sides.append(max_list_consecutive[:stack_len])
        
        # for side in available_sides:
        #     self.robot_pos.t + self.pliers[side].theta 
        return available_sides if available_sides != [] else None

    def angular_distance(self, a1, a2):
        diff = (a2 - a1 + np.pi) % (2 * np.pi) - np.pi
        return abs(diff)

    def get_street_grid_path(self, start, target, margin=0.23):
        import heapq
        
        def pt(x, y): return (round(x, 4), round(y, 4))
        
        start = pt(*start)
        target = pt(*target)
        nodes = set([start, target])
        
        # --- NEW: Use safe boundaries for the street grid ---
        # We add lines around the forbidden zone so the robot can walk AROUND it
        x_lines = [self.safe_x_min, self.f_zone_x_min, self.f_zone_x_max, self.safe_x_max, start[0], target[0]]
        y_lines = [self.safe_y_min, self.f_zone_y_min, self.safe_y_max, start[1], target[1]]
        
        # 1. Add the intersections (ONLY if they are safe)
        for x in x_lines:
            for y in y_lines:
                if self.is_point_safe(x, y):
                    nodes.add(pt(x, y))
                
        # 2. Project Start and Target onto the lines
        for p in [start, target]:
            for x in x_lines:
                if self.is_point_safe(x, p[1]): nodes.add(pt(x, p[1]))
            for y in y_lines:
                if self.is_point_safe(p[0], y): nodes.add(pt(p[0], y))
                    
        nodes = list(nodes)
        edges = {n: [] for n in nodes}
        
        def add_edge(n1, n2):
            if n1 != n2:
                # Verify the edge doesn't cut through the forbidden zone
                if self.is_direct_path_clear(n1, n2, margin=margin):
                    dist = math.hypot(n1[0]-n2[0], n1[1]-n2[1])
                    edges[n1].append((dist, n2))
                    edges[n2].append((dist, n1))

        # 3. Connect nodes that share the same horizontal or vertical "Street"
        for i, n1 in enumerate(nodes):
            for n2 in nodes[i+1:]:
                if n1[0] == n2[0] and n1[0] in x_lines: add_edge(n1, n2)
                elif n1[1] == n2[1] and n1[1] in y_lines: add_edge(n1, n2)

        # 5. Mini-Dijkstra Pathfinding
        queue = [(0, start, [start])]
        visited = set()
        
        while queue:
            cost, current, path = heapq.heappop(queue)
            if current == target:
                cleaned = []
                for p in path:
                    if len(cleaned) >= 2:
                        dx1, dy1 = cleaned[-1][0] - cleaned[-2][0], cleaned[-1][1] - cleaned[-2][1]
                        dx2, dy2 = p[0] - cleaned[-1][0], p[1] - cleaned[-1][1]
                        if abs(dx1*dy2 - dx2*dy1) < 1e-4: cleaned.pop() 
                    cleaned.append(p)
                return cleaned, cost
                
            if current in visited: continue
            visited.add(current)
            
            for d, neighbor in edges[current]:
                if neighbor not in visited:
                    heapq.heappush(queue, (cost + d, neighbor, path + [neighbor]))
                    
        return [], float('inf')

    def is_direct_path_clear(self, start_pos, target_pos, target_crate_id=None, margin=0.23):
        """Checks if the straight line hits crates OR the forbidden zone."""
        x1, y1 = start_pos
        x2, y2 = target_pos
        
        # --- NEW: Quick check if start or end are fundamentally unsafe ---
        if not self.is_point_safe(x1, y1) or not self.is_point_safe(x2, y2):
            return False

        dx = x2 - x1
        dy = y2 - y1
        length = math.hypot(dx, dy)
        
        if length == 0:
            return True
            
        # --- NEW: Sample points along the line to check for forbidden zone intersection ---
        # We sample every 10cm to ensure the line doesn't cross the forbidden zone
        steps = int(length / 0.1)
        if steps > 0:
            for i in range(1, steps):
                tx = x1 + (dx * (i / steps))
                ty = y1 + (dy * (i / steps))
                if not self.is_point_safe(tx, ty):
                    return False # The path cuts through the forbidden zone!

        # --- OLD LOGIC: Check against crates ---
        length_sq = length**2
        for cid, crate in self.haz_crates.items():
            if cid == target_crate_id or crate.state != -1:
                continue

            t = max(0.0, min(1.0, ((crate.x - x1) * dx + (crate.y - y1) * dy) / length_sq))
            closest_x = x1 + t * dx
            closest_y = y1 + t * dy
            
            # Margin is the robot width + crate width + margin (if necessary)
            if math.hypot(crate.x - closest_x, crate.y - closest_y) < margin:
                return False 
                
        return True
    
    def get_best_path(self, target_pos, target_crate_id=None, allow_critical=False):
        """
        - If allow_critical=False: Returns path ONLY if 0.35m margin is respected.
        - If allow_critical=True: Tries 0.35m, then 0.23m, then returns direct line 
        no matter what (Best Effort).
        """
        start_pos = (self.robot_pos.x, self.robot_pos.y)
        IDEAL_MARGIN = 0.22
        CRITICAL_MARGIN = 0.13

        # --- 1. TRY THE SAFE WAY (0.35m) ---
        if self.is_direct_path_clear(start_pos, target_pos, target_crate_id, margin=IDEAL_MARGIN):
            return [start_pos, target_pos], math.hypot(target_pos[0]-start_pos[0], target_pos[1]-start_pos[1])

        path, cost = self.get_street_grid_path(start_pos, target_pos, margin=IDEAL_MARGIN)
        if path:
            return path, cost

        # --- 2. IF NOT ALLOWED TO BE RISKY, STOP HERE ---
        if not allow_critical:
            return [], float('inf')

        # --- 3. ALLOWED TO BE RISKY: TRY CRITICAL (0.23m) ---
        self.get_logger().info("Strict path blocked. Attempting Critical Margin.")
        
        if self.is_direct_path_clear(start_pos, target_pos, target_crate_id, margin=CRITICAL_MARGIN):
            return [start_pos, target_pos], math.hypot(target_pos[0]-start_pos[0], target_pos[1]-start_pos[1])

        path, cost = self.get_street_grid_path(start_pos, target_pos, margin=CRITICAL_MARGIN)
        if path:
            return path, cost
        
        path, cost = self.get_street_grid_path(start_pos, target_pos, margin=0)
        if path:
            return path, cost

        # --- 4. ABSOLUTE FALLBACK (Only if allow_critical=True) ---
        # Even if we might hit a crate, we return the direct line to avoid a 'freeze'
        self.get_logger().warn("No safe path found even with critical margin. Using direct line.")
        return [start_pos, target_pos], math.hypot(target_pos[0]-start_pos[0], target_pos[1]-start_pos[1])

    def navigate_path(self, path, angle):
        """
        Commands the robot to follow the list of waypoints.
        Returns True if reached, False if interrupted.
        """
        if not path:
            self.get_logger().error("Navigation failed: Path is empty!")
            return False

        # Skip the first point if it's the current robot position
        for i, waypoint in enumerate(path[1:]):
            self.get_logger().info(f"Navigating to waypoint {i+1}/{len(path)-1}: {waypoint}")
            
            # We use your existing movement logic (move_to or similar)
            # Assuming move_to handles the PID/Rotation/Linear movement
            self.move_to(Position(x=waypoint[0], y=waypoint[1], t=angle))
            self.wait_for_motion()
            
        return True

def main(args=None):
    """Run main loop."""
    rclpy.init(args=args)
    action_manager_node = ActionManager()
    
    # --- NEW: Spin with the MultiThreadedExecutor ---
    executor = MultiThreadedExecutor()
    executor.add_node(action_manager_node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        action_manager_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()