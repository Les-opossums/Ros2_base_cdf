#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Action Sequencer Node."""

# TODO: Remove ghosts (review does not seem to work)
# TODO: Review the path finder, sometimes the robot goes above known stacks
# TODO: optimize angles for take / release, sometimes does the whoel turn (but dont take in in the cost, just after)
# TODO add a go back to zone before the end
# TODO better handle the cross with an ennemy

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

    def __init__(self, id, x, y, t, rot = False):
        self.id = id
        self.x = x
        self.y = y
        self.theta = t
        self.state = -1
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
        self.end_zone = None
        self.middle_zone = None
        self.match_finished = False
        

        self.x_enn = None
        self.y_enn = None

        self.x_tag = None
        self.y_tag = None

        self.new_pos = 0
        self.pos_obj = None
        self.max_distance = 0.2
        self.latest_camera_msg = None
        self.last_camera_timestamp = time.time()

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
        self.camera_angles = [0.0]

        # --- NEW: Navigation Constraints ---
        self.robot_radius = 0.25
        
        # Safe boundaries (shrunk by robot radius):
        self.safe_x_min = 0.0 + self.robot_radius
        self.safe_x_max = 3.0 - self.robot_radius
        self.safe_y_min = 0.0 + self.robot_radius
        self.safe_y_max = 2.0 - self.robot_radius
        
        # Forbidden Zone (Real: x=[0.6, 2.4], y=[1.55, 2.0])
        # Expanded Forbidden Zone (grown by robot radius):
        self.f_zone_x_min = 0.6 - self.robot_radius  # 0.35
        self.f_zone_x_max = 2.4 + self.robot_radius  # 2.65
        self.f_zone_y_min = 1.55 - self.robot_radius # 1.30
        self.f_zone_y_max = 2.0                      # Capped at top
        self.final_zone = None

    def is_point_safe(self, x, y):
        """Check if a point is within boundaries and outside forbidden zones."""
        # 1. Check outer boundaries
        if not (self.safe_x_min <= x <= self.safe_x_max and self.safe_y_min <= y <= self.safe_y_max):
            return False
            
        # 2. Check forbidden zone
        if (self.f_zone_x_min <= x <= self.f_zone_x_max) and (self.f_zone_y_min <= y <= self.f_zone_y_max):
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
                ("board_config", "small_objects"),
                ("year", 2026),
                ("boundaries", [0.0, 3.0, 0.0, 2.0]),
            ],
        )

        self.boundaries = self.year = (
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
            200,
            self.timer_match_callback,
            callback_group=self.cb_group
        )
        self.timer_backstage = self.create_timer(
            200,
            self.timer_backstage_callback,
            callback_group=self.cb_group
        )
        self.pub_timer = self.create_timer(0.2, self.publish_board_state,callback_group=self.cb_group)
        # self.timer_planner = self.create_timer(
        #     0.5, 
        #     self.continuous_planner_callback, 
        #     callback_group=self.cb_group
        # )
    def _init_move_timer(self):
        self.move_timer = self.create_timer(
            2,
            self.timer_move_callback,
            callback_group=self.cb_group
        )

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
        updates = {'crates': [], 'pliers': [], 'zones': []}

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

        # 4. Only publish if there is actually something new in ANY of the lists!
        if updates['crates'] or updates['pliers'] or updates['zones']:
            msg = String()
            msg.data = json.dumps(updates)
            self.pub_board_state.publish(msg)

    # =========================================================================
    # UNIFIED CAMERA CALLBACK (HUNGARIAN TRACKING WITH ARUCO ID)
    # =========================================================================
    def aruco_callback(self, msg: VisionDataFrame):
        """Continuously save the latest camera frame without processing it."""
        self.latest_camera_msg = msg
        self.last_camera_timestamp = time.time()

    def _extract_color_from_id(self, aruco_id: int) -> int:
        """Map ArUco ID to internal color code."""
        if aruco_id == 47:
            return 0  # Yellow
        elif aruco_id == 36:
            return 1  # Blue
        elif aruco_id == 41:
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
        self.send_raw("LED 20 0 255 0")
        time.sleep(0.3) 
        
        if time.time() - self.last_camera_timestamp > 0.2:
            return

        # 2. Grab the latest message in the buffer
        msg = self.latest_camera_msg
        if msg is None or not msg.object:
            self.get_logger().info("Stare complete: No objects currently visible.")
            return

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
                matched_crate.color = self._extract_color_from_id(det.id)

                # if matched_crate.color in [-1, 2]:

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

        # =====================================================================
        # 5. GHOST CLEANUP (FOV & Range Verification)
        # =====================================================================
        ghost_ids = []
        fov_half = math.radians(65.0 / 2.0) # 65 degrees FOV divided by 2
        max_range = 0.80 # 80cm limit

        for cid, crate in self.haz_crates.items():
            if crate.last_seen == current_time or crate.state != -1:
                continue
                
            # Distance to crate
            dist = math.hypot(crate.x - self.robot_pos.x, crate.y - self.robot_pos.y)
            
            # If it is close enough to see...
            if dist <= max_range:
                angle_to_crate = math.atan2(crate.y - self.robot_pos.y, crate.x - self.robot_pos.x)
                is_in_view = False
                
                # Check if it falls inside any of our 3 camera cones!
                for cam_angle in self.camera_angles:
                    cam_global_angle = self.robot_pos.t + cam_angle
                    diff = self.angular_distance(angle_to_crate, cam_global_angle)
                    
                    if diff <= fov_half:
                        is_in_view = True
                        break
                        
                if is_in_view:
                    # Should be perfectly visible, but wasn't detected!
                    ghost_ids.append(cid)

        # Erase the ghosts
        # for cid in ghost_ids:
        #     self.get_logger().warn(f"GHOST DETECTED: Crate {cid} missing from expected FOV! Erasing.")
        #     ghost_crate = self.haz_crates[cid]
            
        #     if ghost_crate.state != -1 and ghost_crate.state in self.pliers:
        #         self.pliers[ghost_crate.state].state = -1
                    
        #     del self.haz_crates[cid]
        self.send_raw("LED 20 0 0 255")
        self.get_logger().info("Finished staring go back to match.")

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
            v0 = int(data[2])
            v1 = int(data[3])
            p0 = self.pliers[id * 2]
            p1 = self.pliers[id * 2 + 1]
            if p0.is_running:
                p0.is_running = False
                if not v0 and p0.state != -1:
                    self.get_logger().warn(f"The first plier of ID {id} failed.")
                    self.haz_crates[p0.state].state = -1
                    p0.state = -1

            if p1.is_running:
                p1.is_running = False
                if not v1 and p1.state != -1:
                    self.get_logger().warn(f"The second plier of ID {id} failed.")
                    self.haz_crates[p1.state].state = -1
                    p1.state = -1

            if not any(pl.is_running for pl in self.pliers.values()):
                self.pliers_event.set()
    
    def continuous_planner_callback(self):
        """Background thread: Continuously scores every crate and zone on the map."""
        if self.stop or not hasattr(self, 'haz_crates') or getattr(self, 'robot_pos', None) is None:
            return

        with self.data_lock:
            start_pos = (self.robot_pos.x, self.robot_pos.y)
            x_min, x_max, y_min, y_max = self.rect

            # =================================================================
            # A. SCORE ALL PICKUP OPTIONS
            # =================================================================
            # 1. Reset all crate scores to negative infinity
            for crate in list(self.haz_crates.values()):
                crate.pick_reward = float('-inf')
                
            current_stacks = self.generate_current_stacks()
            approach_distance = 0.3

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
                    if not (self.boundaries[0] <= ex <= self.boundaries[1] and self.boundaries[2] <= ey <= self.boundaries[3]):
                        continue
                        
                    target_pos = (ex, ey)
                    
                    # Check Path
                    if self.is_direct_path_clear(start_pos, target_pos, target_crate_id=primary_id):
                        dist = math.hypot(ex - start_pos[0], ey - start_pos[1])
                        path = [start_pos, target_pos]
                    else:
                        path, dist = self.get_street_grid_path(start_pos, target_pos, x_min, x_max, y_min, y_max)
                        if len(path) >= 2 and not self.is_direct_path_clear(path[-2], target_pos, target_crate_id=primary_id):
                            dist = float('inf')

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
                            crate.pick_reward = reward
                            crate.best_pick_path = best_path
                            crate.use_inverted = best_inv
                            crate.is_part_of_stack = stack_ids

            # =================================================================
            # B. SCORE ALL RELEASE OPTIONS
            # =================================================================
            # 1. Check if we even need to release
            if not self.any_plier_used():
                return
                
            for z_id, zone in list(self.zones.items()):
                zone.release_reward = float('-inf')
                
                # Check if occupied
                if len(self.get_all_points_in_zone(z_id)) > 0:
                    continue 
                
                x = zone.x
                y = zone.y
                distance = 0.26
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
                    if not (self.boundaries[0] + 0.2 < pos[0] < self.boundaries[1] - 0.2 and 
                            self.boundaries[2] + 0.2 < pos[1] < self.boundaries[3] - 0.2): 
                        continue
                        
                    target_pos = (pos[0], pos[1])
                    
                    if self.is_direct_path_clear(start_pos, target_pos):
                        dist = math.hypot(pos[0] - start_pos[0], pos[1] - start_pos[1])
                        path = [start_pos, target_pos]
                    else:
                        path, dist = self.get_street_grid_path(start_pos, target_pos, x_min, x_max, y_min, y_max)
                        if len(path) >= 2 and not self.is_direct_path_clear(path[-2], target_pos):
                            dist = float('inf')
                            
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
        if not self.is_ended:
            self.pub_end_of_match.publish(Bool(data=True))
            self.match_finished = True
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
            self.pliers_event.wait(timeout=10.0)

    def send_plier_cmd(self, ids: dict):
        """Compute and send the vacuum gripper command with pair optimization.
        
        ids: dict containing the plier ID as key and the assigned action string as value.
             Example: {2: ["pick", ID crate], 7: ["drop",  ID crate], 8: ["rev_drop", ID crate], 0: "pick"}
             
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

            action1 = ids[current_id][0]
            plier1 = self.pliers[current_id]
            crate1 = self.haz_crates[ids[current_id][1]]

            # Vérifier si c'est un ID pair et si son voisin direct est aussi commandé
            if current_id % 2 == 0 and pair_neighbor in ids:
                action2 = ids[pair_neighbor][0]
                plier2 = self.pliers[pair_neighbor]
                crate2 = self.haz_crates[ids[pair_neighbor][1]]
                
                # Cas A : Les deux pinces ont la même action (ex: pick/pick)
                if action1 == action2:
                    mode = action_map.get(action1, 1) # Fallback to 1 if unknown string
                    self.pub_command.publish(String(data=f"PINCE {cmd_id} {mode} 2"))
                    if mode == 1:
                        plier1.state = crate1.id
                        plier2.state = crate2.id
                        crate1.state = current_id
                        crate2.state = pair_neighbor
                    else:
                        plier1.state = -1
                        plier2.state = -1
                        crate1.state = -1
                        crate2.state = -1
                        
                        # Update pos
                        self.update_crate_pos(plier1, crate1)
                        self.update_crate_pos(plier2, crate2)

                        # Update color if needed
                        if mode == 3:
                            if crate1.color == 1:
                                crate1.color = 0
                            elif crate1.color == 0:
                                crate1.color = 1
                            if crate2.color == 1:
                                crate2.color = 0
                            elif crate2.color == 0:
                                crate2.color = 1

                    # Set plier running
                    plier1.is_running = True
                    plier2.is_running = True

                    # Remove for next check the ids
                    processed_ids.add(current_id)
                    processed_ids.add(pair_neighbor)
                    continue
                
                # Cas B : Combinaison mixte spéciale -> rev_drop(0) et drop(1)
                elif action1 == "rev_drop" and action2 == "drop":
                    self.pub_command.publish(String(data=f"PINCE {cmd_id} 5 2"))
                    plier1.state = -1
                    plier2.state = -1
                    crate1.state = -1
                    crate2.state = -1
                    self.update_crate_pos(plier1, crate1)
                    self.update_crate_pos(plier2, crate2)
                    if crate1.color == 1:
                        crate1.color = 0
                    elif crate1.color == 0:
                        crate1.color = 1
                    plier1.is_running = True
                    plier2.is_running = True
                    processed_ids.add(current_id)
                    processed_ids.add(pair_neighbor)
                    continue
                
                # Cas C : Combinaison mixte spéciale -> drop(0) et rev_drop(1)
                elif action1 == "drop" and action2 == "rev_drop":
                    self.pub_command.publish(String(data=f"PINCE {cmd_id} 4 2"))
                    plier1.state = -1
                    plier2.state = -1
                    crate1.state = -1
                    crate2.state = -1
                    self.update_crate_pos(plier1, crate1)
                    self.update_crate_pos(plier2, crate2)
                    if crate2.color == 1:
                        crate2.color = 0
                    elif crate2.color == 0:
                        crate2.color = 1
                    plier1.is_running = True
                    plier2.is_running = True
                    processed_ids.add(current_id)
                    processed_ids.add(pair_neighbor)
                    continue
                
                # Si actions mixtes non supportées par paire (ex: pick + drop),
                # on laisse le code continuer pour les traiter individuellement.

            # Traitement individuel (ID seul, voisin manquant, ou paire incompatible)
            mode = action_map.get(action1, 1)
            side = current_id % 2
            self.pub_command.publish(String(data=f"PINCE {cmd_id} {mode} {side}"))
            if mode == 1:
                plier1.state = crate1.id
                crate1.state = current_id
            else:
                plier1.state = -1
                crate1.state = -1
                if mode == 3:
                    if crate1.color == 1:
                        crate1.color = 0
                    elif crate1.color == 0:
                        crate1.color = 1
                self.update_crate_pos(plier1, crate1)
            plier1.is_running = True
            processed_ids.add(current_id)

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

    def any_plier_used(self):
        return any(pl.state != -1 for pl in self.pliers.values())

    def generate_current_stacks(self):
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
            available_crates = [c for c in self.haz_crates.values() if c.state == -1]
            
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

    def smart_moves(self):
        self.send_raw("VMAX 0.4")
        self.send_raw("VTMAX 1.5")
        
        activate_check_stack = False
        steal_poses = [[0.45, 0.45], [0.45, 1.1], [2.55, 1.1], [2.55, 0.45]]
        id_steal = 0

        while not self.match_finished:
            self.continuous_planner_callback()
            action = None
            
            # =================================================================
            # 1. INSTANTLY CHOOSE THE BEST ACTION FROM CACHED SCORES
            # =================================================================
            with self.data_lock:
                pliers_used = any(pl.state != -1 for pl in self.pliers.values())
                
                if pliers_used:
                    # Find the highest scoring zone
                    best_zone = max(self.zones.values(), key=lambda z: z.release_reward, default=None)
                    
                    if best_zone and best_zone.release_reward > float('-inf'):
                        rel_path = best_zone.best_release_path
                        best_release_pos = best_zone.best_release_pos
                        action = "RELEASE"
                    else:
                        action = "FINAL_ZONE"
                else:
                    # Find the highest scoring crate
                    best_crate = max(self.haz_crates.values(), key=lambda c: c.pick_reward, default=None)
                    
                    if best_crate and best_crate.pick_reward > float('-inf'):
                        pick_crate_ids = best_crate.is_part_of_stack
                        pick_path = best_crate.best_pick_path
                        is_inv = best_crate.use_inverted
                        action = "PICK"
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

                if len(rel_path) > 1:
                    for wp in rel_path[1:]:
                        self.move_to(Position(wp[0], wp[1], target_angle))
                        self.wait_for_motion()
                self.stare_and_update()

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

                self.send_plier_cmd(id_active_pliers)
                self.wait_for_plier()

                with self.data_lock:
                    for pl_id in id_active_pliers:
                        self.pliers[pl_id].state = -1
                continue

            # =================================================================
            # 3. EXECUTE FINAL ZONE FALLBACK
            # =================================================================
            elif action == "FINAL_ZONE":
                self.get_logger().info("Cannot find release zone. Going to final zone.")
                with self.data_lock:
                    if self.final_zone is None:
                        self.get_logger().info("No final zone defined (no color received). Skipping.")
                        time.sleep(1.0)
                        continue

                self.move_to(Position(self.final_zone["x"], 1.2, 0.0))
                self.wait_for_motion()
                     
                self.move_to(Position(self.final_zone["x"], self.final_zone["y"], 0.0))
                self.wait_for_motion()
                 
                with self.data_lock:
                    id_active_pliers_all = {}
                    for pid, plier in self.pliers.items():
                        if plier.state != -1:
                            id_active_pliers_all[pid] = ["drop", plier.state]
                           
                self.send_plier_cmd(id_active_pliers_all)
                self.wait_for_plier()
                
                with self.data_lock:
                    for pl_id in id_active_pliers_all:
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
                    current_robot_theta = self.robot_pos.t
                
                target_angle = target_pos.t + (np.pi if is_inv else 0.0)
                final_robot_theta = target_angle - pliers_pos.t

                x_offset, y_offset = self.rotate_point(pliers_pos.x, pliers_pos.y, final_robot_theta)
                
                final_pos = Position(
                    target_pos.x - x_offset,
                    target_pos.y - y_offset,
                    final_robot_theta
                )

                entry_point = pick_path[-1] if len(pick_path) > 0 else (self.robot_pos.x, self.robot_pos.y)
                angle_to_crate = math.atan2(target_pos.y - entry_point[1], target_pos.x - entry_point[0])
                
                if activate_check_stack:
                    best_camera_theta = current_robot_theta
                    min_rot = float('inf')
                    for cam_angle in self.camera_angles:
                        req_theta = (angle_to_crate - cam_angle) % (2 * math.pi)
                        if req_theta > math.pi: req_theta -= 2 * math.pi
                        
                        rot = self.angular_distance(current_robot_theta, req_theta)
                        if rot < min_rot:
                            min_rot = rot
                            best_camera_theta = req_theta

                if len(pick_path) > 1:
                    for i, wp in enumerate(pick_path[1:]):
                        if activate_check_stack and i == len(pick_path[1:]) - 1:
                            self.move_to(Position(wp[0], wp[1], best_camera_theta))
                        else:
                            self.move_to(Position(wp[0], wp[1], final_robot_theta))
                        self.wait_for_motion()
                self.stare_and_update()
                self.centering()

                if activate_check_stack:
                    is_ghost = False
                    with self.data_lock:
                        for cid in pick_crate_ids:
                            if cid not in self.haz_crates:
                                is_ghost = True
                                break
                    if is_ghost:
                        self.get_logger().warn("Crate missing after approach! Aborting pick...")
                        continue 

                if not activate_check_stack or abs(self.angular_distance(best_camera_theta, final_robot_theta)) > 0.05:
                    self.move_to(Position(entry_point[0], entry_point[1], final_robot_theta))
                    # self.get_logger().info(f'Entry Point {entry_point[0]}, {entry_point[1]}, {final_robot_theta}')
                    self.wait_for_motion()

                self.move_to(final_pos)
                # self.get_logger().info(f'Final Pose {final_pos.x}, {final_pos.y}, {final_robot_theta}')
                self.wait_for_motion()

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

                self.send_plier_cmd(dict_sel_pliers)
                self.wait_for_plier()
                continue

            # =================================================================
            # 5. EXECUTE EXPLORE / STEAL
            # =================================================================
            elif action == "EXPLORE":
                self.get_logger().info("Nothing to do: Exploring / Stealing...")
                pos = steal_poses[id_steal % len(steal_poses)]
                
                with self.data_lock:
                    in_bounds = (self.boundaries[0] + 0.25 < pos[0] < self.boundaries[1] - 0.25 and 
                                 self.boundaries[2] + 0.25 < pos[1] < self.boundaries[3] - 0.25)
                
                if in_bounds:
                    self.move_to(Position(pos[0], pos[1], (id_steal * 2.3998) % (2 * np.pi)))
                    self.wait_for_motion()
                    self.stare_and_update()
                    
                id_steal += 1
                time.sleep(0.1)

    def get_safe_perimeter_path(self, target_pos):
        """
        Finds the shortest path along a safe rectangle perimeter to a target.
        
        robot_pos: (x, y) tuple of the robot
        target_pos: (x, y) tuple of the object to grab
        rect: (x_min, x_max, y_min, y_max) tuple defining the safe perimeter
        """
        x_min, x_max, y_min, y_max = self.rect
        cx, cy = (x_min + x_max) / 2.0, (y_min + y_max) / 2.0  # Center of rect
        robot_pos = (self.robot_pos.x, self.robot_pos.y) 
        # 1. Project a point to the nearest edge of the rectangle
        def project_to_edge(x, y):
            px = max(x_min, min(x, x_max))
            py = max(y_min, min(y, y_max))
            # If strictly inside the rectangle, push it to the nearest edge
            if x_min < px < x_max and y_min < py < y_max:
                dists = {
                    (x_min, py): px - x_min,
                    (x_max, py): x_max - px,
                    (px, y_min): py - y_min,
                    (px, y_max): y_max - py
                }
                px, py = min(dists, key=dists.get)
            return round(px, 4), round(py, 4)

        # 2. Find where Robot ENTERS the ring, and where it DIVES for the object
        entry = project_to_edge(*robot_pos)
        dive = project_to_edge(*target_pos)

        # 3. Gather all points on our Ring Road and remove duplicates
        perimeter_points = [
            (x_min, y_max), (x_max, y_max), # Top-Left, Top-Right
            (x_max, y_min), (x_min, y_min), # Bottom-Right, Bottom-Left
            entry, dive
        ]
        perimeter_points = list(set(perimeter_points)) 
        
        # 4. The Math Trick: Sort points by their angle from the center!
        # This perfectly orders them counter-clockwise around the rectangle.
        perimeter_points.sort(key=lambda p: math.atan2(p[1] - cy, p[0] - cx))
        
        i_entry = perimeter_points.index(entry)
        i_dive = perimeter_points.index(dive)
        n = len(perimeter_points)
        
        # 5. Extract the two possible paths (Clockwise & Counter-Clockwise)
        path_ccw, path_cw = [], []
        
        i = i_entry
        while True:
            path_ccw.append(perimeter_points[i])
            if i == i_dive: break
            i = (i + 1) % n  # Step forward
            
        i = i_entry
        while True:
            path_cw.append(perimeter_points[i])
            if i == i_dive: break
            i = (i - 1) % n  # Step backward

        # 6. Calculate total distances to pick the shortest one
        def calc_dist(path):
            return sum(math.hypot(path[j][0]-path[j-1][0], path[j][1]-path[j-1][1]) for j in range(1, len(path)))

        best_perimeter_path = path_ccw if calc_dist(path_ccw) < calc_dist(path_cw) else path_cw

        # 7. Add the starting robot position and final object position
        final_path = [robot_pos] + best_perimeter_path + [target_pos]
        
        # Clean up redundant points (e.g., if robot is already on the entry point)
        cleaned_path = []
        for p in final_path:
            if not cleaned_path or cleaned_path[-1] != p:
                cleaned_path.append(p)
                
        return cleaned_path, calc_dist(cleaned_path)

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
            
    def is_any_point_in_zone(self, id_zone):
        """
        Checks if any haz_crate is currently sitting inside the specified zone.
        
        Args:
            id_zone: The integer/string ID of the zone in self.zones
            
        Returns:
            True if at least one crate is in the zone, False otherwise.
        """
        if id_zone not in self.zones:
            self.get_logger().warn(f"Zone ID {id_zone} does not exist!")
            return False
            
        # 1. Grab the specific zone object
        zone = self.zones[id_zone]
        
        # 2. Calculate its boundaries once
        half_size = zone.size / 2.0
        min_x = zone.x - half_size
        max_x = zone.x + half_size
        min_y = zone.y - half_size
        max_y = zone.y + half_size
        
        # 3. Check every crate (We use .values() to get the actual HazCrate objects)
        for crate in self.haz_crates.values():
            if (min_x <= crate.x <= max_x) and (min_y <= crate.y <= max_y) and crate.state == -1:
                return True  # Found one! Exit immediately.
                
        return False # Looked at all crates, none were in the zone
    
    def get_all_points_in_zone(self, id_zone):
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
            for c in self.haz_crates.values():
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
        coeff_enn = -5 
        
        val_center = (1.5 - px) ** 2 + (1 - py) ** 2
        # Use the TRUE path distance, not just a straight line guess!
        val_dst = path_distance**2 
        
        if self.x_enn is not None:
            val_ennemi = (self.x_enn - px) ** 2 + (self.y_enn - py) ** 2
        else:
            val_ennemi = 0
            
        return (coeff_dst * val_dst) + (coeff_enn * val_ennemi) + (coeff_center * val_center)


    def compute_release_rewards(self):
        if not self.any_plier_used():
            return None, None, []
            
        max_reward = float('-inf')
        best_zone_id = None
        best_pos = None
        best_path = []
        
        start_pos = (self.robot_pos.x, self.robot_pos.y)
        x_min, x_max, y_min, y_max = self.rect

        with self.data_lock: # Lock data while we check paths
            for z_id, zone in self.zones.items():
                if len(self.get_all_points_in_zone(z_id)) > 0:
                    continue # Zone is occupied
                
                x = zone.x
                y = zone.y
                distance = 0.26
                av_poses = [
                    [x + distance, y, 3.14],
                    [x - distance, y, 0.0],
                    [x, y + distance, 4.71],
                    [x, y - distance, 1.57],
                ]

                for pos in av_poses:
                    # Boundary check
                    if not (self.boundaries[0] + 0.2 < pos[0] < self.boundaries[1] - 0.2 and 
                            self.boundaries[2] + 0.2 < pos[1] < self.boundaries[3] - 0.2): 
                        continue
                        
                    target_pos = (pos[0], pos[1])
                    
                    # 1. Physically check the path to this specific side!
                    if self.is_direct_path_clear(start_pos, target_pos):
                        dist = math.hypot(pos[0] - start_pos[0], pos[1] - start_pos[1])
                        path = [start_pos, target_pos]
                    else:
                        path, dist = self.get_street_grid_path(start_pos, target_pos, x_min, x_max, y_min, y_max)
                        # Security Check: Dive blocked?
                        if len(path) >= 2 and not self.is_direct_path_clear(path[-2], target_pos):
                            dist = float('inf')
                            
                    # If this side is completely blocked, skip it!
                    if dist == float('inf'):
                        continue

                    # 2. Score the position using the ACTUAL path distance
                    reward = self.compute_release_penality(pos[0], pos[1], dist)

                    # 3. Update the global champion
                    if reward > max_reward:
                        max_reward = reward
                        best_zone_id = z_id
                        best_pos = pos
                        best_path = path
                        
        return best_zone_id, best_pos, best_path

    def compute_pick_penality(self, x, y):
        coeff_center = -1 # -0.005
        coeff_dst = -1
        coeff_enn = -4 # 0.05
        # coeef_end = -0.0001
        val_center = (1.5 - x) ** 2 + (1 - y) ** 2
        val_dst = (self.robot_pos.x - x) ** 2 + (self.robot_pos.y - y) ** 2
        if self.x_enn is not None:
            val_ennemi = (self.x_enn - x) ** 2 + (self.y_enn - y) ** 2
        else:
            val_ennemi = 0
        return coeff_dst * val_dst + coeff_enn * val_ennemi + coeff_center * val_center

    def compute_pick_rewards(self):
        if not self.any_plier_side_available():
            return None, None, []

        min_total_dist = float('inf')
        best_target_ids = None  # Replaces best_crate_id and target_stack_id
        use_inverted_approach = False
        best_path = []
        
        approach_distance = 0.3
        start_pos = (self.robot_pos.x, self.robot_pos.y)
        x_min, x_max, y_min, y_max = self.rect

        with self.data_lock:
            # 1. Grab the freshest stacks right now!
            # Returns a list of ID lists: e.g., [[1], [2, 3], [4]]
            current_stacks = self.generate_current_stacks()

            # 2. Iterate through the stacks instead of individual crates
            for stack_ids in current_stacks:
                
                # Fetch the actual crate objects for this stack
                group_crates = [self.haz_crates[cid] for cid in stack_ids if cid in self.haz_crates]
                if not group_crates:
                    continue

                # 3. Security Filters
                # If ANY crate in this stack is in a safe zone, skip the entire stack
                skip_stack = False
                for crate in group_crates:
                    if self.get_current_zone(crate.x, crate.y, crate.theta) is not None and (crate.color in (-1, self.color, 2)) or self.in_final_zone(crate.x, crate.y, crate.theta):
                        skip_stack = True
                        break
                
                if skip_stack:
                    continue
                
                # 4. Consolidation (Math works perfectly whether it's 1 crate or 3!)
                mean_x = np.mean([c.x for c in group_crates])
                mean_y = np.mean([c.y for c in group_crates])
                mean_theta = np.arctan2(np.sum([np.sin(c.theta) for c in group_crates]), 
                                        np.sum([np.cos(c.theta) for c in group_crates]))

                # 5. Calculate Entry Points
                entries = [
                    (mean_x - approach_distance * np.cos(mean_theta), mean_y - approach_distance * np.sin(mean_theta), False),
                    (mean_x + approach_distance * np.cos(mean_theta), mean_y + approach_distance * np.sin(mean_theta), True)
                ]

                for ex, ey, is_inv in entries:
                    if not (self.boundaries[0] <= ex <= self.boundaries[1] and self.boundaries[2] <= ey <= self.boundaries[3]):
                        continue
                        
                    target_pos = (ex, ey)
                    
                    # Pass the primary crate ID to ignore collisions with the target itself
                    primary_id = stack_ids[0]
                    
                    # 6. Check Path!
                    if self.is_direct_path_clear(start_pos, target_pos, target_crate_id=primary_id):
                        dist = math.hypot(ex - start_pos[0], ey - start_pos[1])
                        path = [start_pos, target_pos]
                    else:
                        path, dist = self.get_street_grid_path(start_pos, target_pos, x_min, x_max, y_min, y_max)
                        # Security Check: Dive blocked?
                        if len(path) >= 2 and not self.is_direct_path_clear(path[-2], target_pos, target_crate_id=primary_id):
                            dist = float('inf')

                    # 7. Save the absolute best one
                    if dist < min_total_dist:
                        min_total_dist = dist
                        best_target_ids = stack_ids  # Save the whole list of IDs!
                        use_inverted_approach = is_inv
                        best_path = path
                            
        return best_target_ids, use_inverted_approach, best_path

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
            if max_consecutive == 4:
            # if max_consecutive >= stack_len:
                available_sides.append(max_list_consecutive[:stack_len])
        
        # for side in available_sides:
        #     self.robot_pos.t + self.pliers[side].theta 
        return available_sides if available_sides != [] else None

    def angular_distance(self, a1, a2):
        diff = (a2 - a1 + np.pi) % (2 * np.pi) - np.pi
        return abs(diff)

    def any_plier_side_available(self):
        for side in range(4):
            if any(self.pliers[pl_id].state >= 0 for pl_id in range(side * 4, (side + 1) * 4)):
                continue
            return True
        return False

    def get_street_grid_path(self, start, target, x_min, x_max, y_min, y_max):
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
                if self.is_direct_path_clear(n1, n2):
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

    def get_best_pickup_target(self, objects_to_check, x_min, x_max, y_min, y_max, offset_dist=0.23):
        """
        Evaluates all objects, checks both entry points, respects boundaries, 
        and calculates the absolute shortest path.
        """
        best_crate_id = None
        best_path = []
        min_total_dist = float('inf')
        best_is_inverted = False

        start_pos = (self.robot_pos.x, self.robot_pos.y)

        for obj in objects_to_check:
            # 1. Calculate the two entry points (Front and Back)
            entries = [
                (obj.x - offset_dist * math.cos(obj.theta), obj.y - offset_dist * math.sin(obj.theta), False), # Standard
                (obj.x + offset_dist * math.cos(obj.theta), obj.y + offset_dist * math.sin(obj.theta), True)   # Inverted (pi)
            ]
            
            for ex, ey, is_inv in entries:
                # 2. Filter out entry points that go out of the board limits!
                if not (self.boundaries[0] <= ex <= self.boundaries[1] and 
                        self.boundaries[2] <= ey <= self.boundaries[3]):
                    continue 
                    
                target_pos = (ex, ey)
                
                # 3. Check if we have a direct straight-line path
                if self.is_direct_path_clear(start_pos, target_pos, target_crate_id=obj.id):
                    dist = math.hypot(ex - start_pos[0], ey - start_pos[1])
                    path = [start_pos, target_pos]
                else:
                    # 4. Fallback to the Street Grid path
                    path, dist = self.get_street_grid_path(start_pos, target_pos, x_min, x_max, y_min, y_max)
                    
                    # 5. Security Check: Ensure the final "dive" from the grid to the crate isn't blocked!
                    if len(path) >= 2 and not self.is_direct_path_clear(path[-2], target_pos, target_crate_id=obj.id):
                        dist = float('inf') # Blocked by other crates, discard this entry point
                
                # 6. Keep the absolute shortest path
                if dist < min_total_dist:
                    min_total_dist = dist
                    best_path = path
                    best_crate_id = obj.id
                    best_is_inverted = is_inv

        return best_crate_id, best_path, best_is_inverted, min_total_dist

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
            
            if math.hypot(crate.x - closest_x, crate.y - closest_y) < margin:
                return False 
                
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