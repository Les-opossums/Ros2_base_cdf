#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Action Sequencer Node."""

# TODO: optimize angles for take / release, sometimes does the whoel turn (but dont take in in the cost, just after)
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
)

import threading
from threading import Lock, Event, Thread

import time
import os
import json
from dataclasses import dataclass
from ament_index_python.packages import get_package_share_directory
import numpy as np

from opossum_action_sequencer.scripts.StateMachine import (
    GlobalSM,
    PickSM,
    ReleaseSM,
    GoHomeSM,
    ExploreSM,
    StopSM,
    CursorSM,
    GoBoardSM,
)

def are_angles_close(angle1, angle2, tolerance=0.01):
    # This trick naturally handles the circle wrap-around
    diff = np.angle(np.exp(1j * (angle1 - angle2)))
    return abs(diff) <= tolerance

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

@dataclass
class Camera:
    id: int
    angle: float
    cone_range: float
    distance_range: float
    last_timestamp: float
    last_msg: VisionDataFrame

    def __init__(self, id, angle, cone_range = 1.2, distance_range = 1.2):
        self.id = id
        self.angle = angle
        self.cone_range = cone_range
        self.distance_range = distance_range
        self.last_timestamp = time.time()
        self.last_msg = None

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

        self.enemy_pos = None
        self.time_stare = 0.15 # s

        self.pos_obj = None
        self.max_distance = 0.2
        # self.cameras = {1: Camera(1, 0.0), 2: Camera(2, 1.57)}
        self.cameras = {1: Camera(1, 0.0), 2: Camera(2, 1.57), 3: Camera(3, 4.71)}
        self.latest_camera_msg = {1: None, 2: None, 3: None}
        self.last_camera_timestamp = {1: time.time(), 2: time.time(), 3: time.time()}

        # Action Done
        self.is_robot_moving = False
        self.is_robot_arrived = False
        self.is_pump_top_on = False
        self.is_pump_bottom_on = False
        self.obstacle_detected = False
        self.robot_pos = None
        self.motion_done = True
        self.break_engage = False
        self.pliers_done = True
        self.color = None
        self.last_time_pliers = time.time()
        self.last_time_move = time.time()
        self.last_enemy_detected = time.time() - 20.0

        # Process
        self.motion_done_event = Event()
        self.end_match_event = Event()
        self.data_lock = threading.RLock()
        self._lock_gsm = Lock()
        self._stop_event = Event()
        self.loop_frequency = 10.0
        self.zone_cursor_pick = [1.1, 0.175]
        self.zone_cursor_release_id = 1
        self.entry_zone = [0.5, 1.1]
        self.cursor_in = [1.35, 0.2, 1.54]
        self.cursor_out = [0.73, 0.2, 1.54]
        self.pince_cursor = 4
        self.steal_poses = [[0.65, 1.10], [0.90, 1.10], [1.20, 1.10], [1.50, 1.10], [1.80, 1.10], [2.10, 1.10], [2.35, 1.10], 
                            [0.65, 0.80],                                                                       [2.35, 0.80],
                            [0.65, 0.49], [0.90, 0.49], [1.20, 0.49], [1.50, 0.49], [1.80, 0.49], [2.10, 0.49], [2.35, 0.49],]

        # --- NEW: Navigation Constraints ---
        self.robot_radius = 0.2
        self.classic_margin = 0.3
        self.critical_margin = 0.15

        # Safe boundaries (shrunk by robot radius):
        self.safe_x_min = self.boundaries[0] + self.robot_radius
        self.safe_x_max = self.boundaries[1] - self.robot_radius
        self.safe_y_min = self.boundaries[2] + self.robot_radius
        self.safe_y_max = self.boundaries[3] - self.robot_radius
        
        # Forbidden Zone (Real: x=[0.6, 2.4], y=[1.55, 2.0])
        # Expanded Forbidden Zone (grown by robot radius):
        self.f_zone_y_min = 1.55 - self.robot_radius # 1.30
        self.final_zone = None

        self.init_match()

    def gaussienne(self, t):
        sigma = 10
        center = self.match_time / 2
        return self.amp_gauss * np.exp(-((t - center)**2) / (2 * sigma**2)) - self.amp_gauss / 2

    @property
    def coeff_release(self):
        add_end = 500 if self.release_end else 0
        return -self.gaussienne(time.time() - self.start_match_time) + add_end

    @property
    def coeff_pick(self):
        return self.gaussienne(time.time() - self.start_match_time)
    
    def init_match(self):
        """Réinitialise l'état du robot et la carte pour un nouveau match (et sert d'init)."""
        
        # Vérifie si le thread existe (donc si on est en train de RESET) et s'il tourne
        if hasattr(self, '_thread') and self._thread.is_alive():
            self.get_logger().info("Réinitialisation du match en cours...")

            # 1. Forcer l'arrêt physique et logiciel
            self.stop_script()
            self.stop_state_machine()

        with self.data_lock:
            # 2. Réinitialiser les drapeaux de fonctionnement
            self.is_started = False
            self.is_ended = False
            self.stop = False
            self.arrived_home = False
            self.obstacle_detected = False
            self.cursor_begin_done = False
            self.cursor_end_done = False
            self.activate_cursor = False
            self.release_end = False
            
            # 3. Libérer les événements de synchronisation
            self.motion_done = True
            self.break_engage = False
            self.motion_done_event.set()
            self.pliers_done = True
            self.end_match_event.clear()

            # 4. Réinitialiser la machine à états de haut niveau
            self.global_sm = GlobalSM.NOP
            self.sub_sm = GlobalSM.NOP
            self.count = 0

            # 5. Recharger la configuration initiale depuis le fichier YAML
            self._init_mapping()
            
            # 6. Réinitialiser l'historique d'envoi pour forcer la maj de l'IHM
            self.last_sent_state = {'crates': {}, 'pliers': {}, 'zones': {}}

        # Il faut réinstancier l'objet Thread pour le prochain start_state_machine().
        self._thread = threading.Thread(target=self._run_loop, daemon=True)

        config_file = os.path.join(
            get_package_share_directory("opossum_bringup"), "config", "match_params.yaml"
        )

        data = yaml.safe_load(open(config_file, "r"))

        self.match_time = data["match_time"] # s

        self.coeff_release_center = data["coeff_release_center"] # Distance to center
        self.coeff_release_dst = data["coeff_release_dst"] # Distance to zone
        self.coeff_release_enn = data["coeff_release_enn"] # Release close to enemi

        self.coeff_pick_center = data["coeff_pick_center"] # Distance to the center
        self.coeff_pick_dst = data["coeff_pick_dst"] # Distance to the stack
        self.coeff_pick_enn = data["coeff_pick_enn"] # Distance to enemy
        self.coeff_critical_level = data["coeff_critical_level"] # Path more critical -> level high
        self.coeff_pick_balance = data["coeff_pick_balance"] # If more blau than yellow in a zone
        self.coeff_pick_num = data["coeff_pick_num"] # Number of stacks
        self.amp_gauss = data["amp_gauss"] # Gauss amp
        self.coeff_far_zone = data["coeff_far_zone"] # Coeff far zone
        self.end_far_zone = False # Coeff far zone
        self.get_logger().info("Match (ré)initialisé avec succès. En attente du LEASH...")    

    def is_point_safe(self, x, y):
        """Check if a point is within boundaries and outside forbidden zones."""
        # 1. Check outer boundaries
        if not (self.safe_x_min <= x <= self.safe_x_max and self.safe_y_min <= y <= self.safe_y_max):
            return False
            
        # 2. Check forbidden zone
        if self.f_zone_y_min <= y:
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
                ("service_reset_match", "reset_match")
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
        self.service_reset_match = (
            self.get_parameter("service_reset_match")
            .get_parameter_value()
            .string_value
        )

    def _init_timers(self):
        """Initialize the timers of the node."""
        # pass
        self.pub_timer = self.create_timer(0.2, self.publish_board_state,callback_group=self.cb_group)

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

        self.obstacle_detected_sub = self.create_subscription(
            Bool,
            "obstacle_detected",
            self.obstacle_detected_callback,
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

        self.reset_match_srv = self.create_service(
            Trigger,
            self.service_reset_match,
            self.reset_match_callback  # Nom de la fonction de callback
        )

    def reset_match_callback(self, request, response):
        """Callback pour déclencher la réinitialisation via un service ROS."""
        try:
            self.init_match()  # Appelle ta logique de reset/init
            response.success = True
            response.message = "Robot et carte réinitialisés avec succès (Prêt pour LEASH)."
        except Exception as e:
            response.success = False
            response.message = f"Erreur lors du reset : {str(e)}"
            self.get_logger().error(f"Échec du reset_match : {e}")
            
        return response

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
        if not hasattr(self, 'haz_crates') or not hasattr(self, 'pliers') or not hasattr(self, 'zones'):
            return

        # --- ADD 'deleted_crates' to updates ---
        updates = {'crates': [], 'pliers': [], 'zones': [], 'morbidity': [], 'deleted_crates': []}

        if 'zones' not in self.last_sent_state:
            self.last_sent_state['zones'] = {}
        if 'crates' not in self.last_sent_state:
            self.last_sent_state['crates'] = {}

        # ==========================================
        # 1. CHECK FOR DELETED CRATES (NEW LOGIC)
        # ==========================================
        current_crate_ids = set(self.haz_crates.keys())
        last_crate_ids = set(self.last_sent_state['crates'].keys())
        
        # If an ID is in our last sent state, but no longer in the map, it was deleted!
        for cid in last_crate_ids - current_crate_ids:
            updates['deleted_crates'].append(cid)
            del self.last_sent_state['crates'][cid] # Remove it from history

        # 2. Check Crates for changes
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
        if updates['crates'] or updates['pliers'] or updates['zones'] or updates['morbidity'] or updates['deleted_crates']:
            msg = String()
            msg.data = json.dumps(updates)
            self.pub_board_state.publish(msg)

    # =========================================================================
    # UNIFIED CAMERA CALLBACK (HUNGARIAN TRACKING WITH ARUCO ID)
    # =========================================================================
    def aruco_callback(self, msg: VisionDataFrame):
        """Continuously save the latest camera frame without processing it."""
        self.cameras[msg.id].last_msg = msg
        self.cameras[msg.id].last_timestamp = time.time()

    def obstacle_detected_callback(self, msg):
        """Continuously save the latest camera frame without processing it."""
        if not self.obstacle_detected and bool(msg.data):
            self.last_enemy_detected = time.time()
        
        self.obstacle_detected = bool(msg.data)

    def _extract_color_from_id(self, aruco_id: int) -> int:
        """Map ArUco ID to internal color code."""
        if aruco_id == 47:
            return 0  # Yellow
        elif aruco_id == 36:
            return 1  # Blue
        elif aruco_id == 41:
            return 2  # Rot (Red)
        return -1 # Unknown

    def stare_and_update(self, crate_dict, camera_target=None):
        """
        Stop, let the camera settle, and process the latest frame in World Coordinates.
        Updates the global map and returns a list of crates seen ONLY by the camera_target.
        """
        if getattr(self, 'robot_pos', None) is None:
            self.get_logger().warn("Stare failed: Robot position unknown.")
            return []

        if camera_target is None:
            camera_target = []
        self.get_logger().debug(f"Staring... focusing on camera {camera_target}")
        
        # 1. Wait for physical motion blur to clear
        current_time = time.time()
        crates_seen_by_target = []
        ids_seen_this_tick = set() # Track what we actually see right now

        for key, cam_obj in self.cameras.items():
            msg = cam_obj.last_msg
            # Check if message is stale
            if msg is None or (current_time - cam_obj.last_timestamp > 0.4):
                continue

            # 2. Grab the latest message in the buffer
            if not msg.object:
                self.get_logger().debug(f"Stare complete: No objects currently visible for camera {key}.")
                continue

            # =====================================================================
            # 3. TRANSFORM DETECTIONS: ROBOT FRAME -> WORLD FRAME
            # =====================================================================
            current_frame_world_dets = []
            cos_t = math.cos(self.robot_pos.t)
            sin_t = math.sin(self.robot_pos.t)

            for det in msg.object:
                # Filter noise/chassis (too close) and far objects
                if det.x ** 2 + det.y ** 2 < 0.05 and det.z > 0.17:
                    continue
                if det.x ** 2 + det.y ** 2 > 1.0 ** 2:
                    continue
                    
                world_det = SimpleNamespace()
                world_det.id = det.id
                world_det.x = self.robot_pos.x + (det.x * cos_t - det.y * sin_t)
                world_det.y = self.robot_pos.y + (det.x * sin_t + det.y * cos_t)
                world_det.theta = self.robot_pos.t + det.theta
                current_frame_world_dets.append(world_det)

            # =====================================================================
            # 4. HUNGARIAN TRACKING LOGIC (Global Update)
            # =====================================================================
            crate_ids = list(crate_dict.keys())
            matched_detection_indices = set()
            
            if crate_ids and current_frame_world_dets:
                num_crates = len(crate_ids)
                num_detections = len(current_frame_world_dets)
                cost_matrix = np.zeros((num_crates, num_detections))

                for i, cid in enumerate(crate_ids):
                    crate = crate_dict[cid]
                    for j, det in enumerate(current_frame_world_dets):
                        dist = math.hypot(crate.x - det.x, crate.y - det.y)
                        cost_matrix[i, j] = dist

                crate_indices, detection_indices = linear_sum_assignment(cost_matrix)

                for crate_idx, det_idx in zip(crate_indices, detection_indices):
                    distance = cost_matrix[crate_idx, det_idx]

                    # Ensure max_distance is defined (e.g., 15cm)
                    max_dist = getattr(self, 'max_distance', 0.15) 
                    
                    if distance <= max_dist:
                        cid = crate_ids[crate_idx]
                        matched_crate = crate_dict[cid]
                        det = current_frame_world_dets[det_idx]

                        # Update existing crate
                        matched_crate.x = det.x
                        matched_crate.y = det.y
                        matched_crate.theta = det.theta
                        matched_crate.last_seen = current_time
                        matched_crate.color = self._extract_color_from_id(det.id)
                        
                        matched_detection_indices.add(det_idx)
                        ids_seen_this_tick.add(cid)
                        
                        # If this crate was seen by the camera we care about, save it!
                        if key in camera_target:
                            crates_seen_by_target.append(matched_crate)

            # =====================================================================
            # 5. NEW DISCOVERY
            # =====================================================================
            for j, det in enumerate(current_frame_world_dets):
                if j not in matched_detection_indices:
                    new_id = max(crate_dict.keys(), default=0) + 1
                    color_val = self._extract_color_from_id(det.id)
                    new_crate = HazCrate(new_id, det.x, det.y, det.theta, rot=(color_val == 2))
                    new_crate.color = color_val
                    new_crate.last_seen = current_time
                    crate_dict[new_id] = new_crate
                    
                    ids_seen_this_tick.add(new_id)
                    
                    # If the new crate was discovered by our target camera, save it!
                    if key in camera_target:
                        crates_seen_by_target.append(new_crate)
                        
                    self.get_logger().debug(f"Tracking: Discovered new crate! ID: {new_id} at X:{det.x:.2f} Y:{det.y:.2f} via Cam {key}")

        # =====================================================================
        # 6. GHOST CLEANUP (Targeted by FOV)
        # =====================================================================  

        for ct in camera_target:          
            if ct in self.cameras.keys():
                to_delete = []
                cam_global_angle = self.robot_pos.t + self.cameras[ct].angle
                fov_half = math.radians(65.0 / 2.0)
                max_range = 1.0 # We ignore detections further than 60cm anyway

                for cid, crate in crate_dict.items():
                    if crate.state != -1: 
                        continue # Do not delete crates currently held in pliers
                    
                    # Is it close enough to be seen?
                    dist = math.hypot(crate.x - self.robot_pos.x, crate.y - self.robot_pos.y)
                    
                    if dist <= max_range and cid not in ids_seen_this_tick:
                        angle_to_crate = math.atan2(crate.y - self.robot_pos.y, crate.x - self.robot_pos.x)
                        
                        # Check if the crate falls inside the target camera's cone
                        diff = getattr(self, 'angular_distance', lambda a, b: abs((a - b + math.pi) % (2 * math.pi) - math.pi))(angle_to_crate, cam_global_angle)
                        
                        if abs(diff) <= fov_half:
                            # Should be perfectly visible to the target camera, but wasn't detected!
                            to_delete.append(cid)
                
                # Execute cleanup
                for cid in to_delete:
                    self.get_logger().debug(f"Ghost cleanup: Crate {cid} vanished from camera {ct}'s FOV. Deleting.")
                    del crate_dict[cid]

        return crates_seen_by_target
        
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
        if self.color == 1:
            self.entry_zone[0] = self.boundaries[1] - self.entry_zone[0] 
            self.cursor_in[0] = self.boundaries[1] - self.cursor_in[0]
            self.cursor_out[0] = self.boundaries[1] - self.cursor_out[0]
            self.zone_cursor_pick[0] = self.boundaries[1] - self.zone_cursor_pick[0]
            self.zone_cursor_release_id = 7
            self.pince_cursor = 5

    def feedback_callback(self, msg):
        """Receive the data from Zynq."""
        if msg.data.startswith("LEASH") and self.ready and not self.is_started:
            self._init_timers()
            self.is_started = True
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
        
        elif msg.data.strip() == "Break,done":
            self.break_engage = True
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
                            self.get_logger().debug(f"The {label} plier of ID {cmd_id} failed.")
                            
                            # Reset tracking
                            crate.state = -1
                            crate.attempts += 1
                            plier.state = -1

            # Global check to release the event loop
            if not any(pl.is_running for pl in self.pliers.values()):
                self.pliers_done = True
    
    def compute_and_update_rewards(self):
        """Background thread: Continuously scores every crate and zone on the map."""
        if self.stop or not hasattr(self, 'haz_crates') or getattr(self, 'robot_pos', None) is None:
            return

        max_pliers = 4
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
                if side == 2:
                    pliers_active += 1
                    continue

                if any(self.pliers[pid].state != -1 for pid in range(side * 4, (side + 1) * 4)):
                    pliers_active += 1
            
            # Check if more than 2
            if pliers_active < max_pliers:
                self.update_pick_reward()

            if not all(pl.state == -1 for pl in self.pliers.values()):
                self.update_release_reward()

    def update_pick_reward(self):
        current_stacks = self.generate_stacks(self.haz_crates)
        approach_distance = 0.32

        for stack_ids in current_stacks:
            group_crates = [self.haz_crates[cid] for cid in stack_ids if cid in self.haz_crates]
            if not group_crates:
                continue

            skip_stack = False
            balance = 0
            for crate in group_crates:
                if self.in_final_zone(crate.x, crate.y, crate.theta):
                    skip_stack = True
                    break
                
                if self.get_current_zone(crate.x, crate.y, crate.theta) is not None:
                    if crate.color == self.color:
                        balance += 1
                    
                    if crate.color == 1 - self.color:
                        balance -= 1

            if balance > 0:
                skip_stack = True

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
            best_critical = 5

            for ex, ey, is_inv in entries:
                if not (self.boundaries[0] + self.robot_radius <= ex <= self.boundaries[1] - self.robot_radius and self.boundaries[2] + self.robot_radius <= ey <= self.boundaries[3] - self.robot_radius):
                    continue
                    
                target_pos = (ex, ey)
                
                # Check Path
                path, dist, critical_level = self.get_best_path(target_pos, target_crate_ids=stack_ids, allow_critical=False)

                if critical_level <= best_critical and dist < best_dist:
                    best_dist = dist
                    best_path = path
                    best_inv = is_inv
                    best_critical = critical_level

            # Write the final score into ALL crates in this stack
            if best_dist != float('inf'):
                reward = self.compute_pick_penality(mean_x, mean_y, len(group_crates), best_dist, best_critical, balance)
                for cid in stack_ids:
                    if cid in self.haz_crates:
                        crate = self.haz_crates[cid]
                        crate.pick_reward = reward + (-15) * crate.attempts
                        crate.best_pick_path = best_path
                        crate.use_inverted = best_inv
                        crate.is_part_of_stack = stack_ids

    def update_release_reward(self):
        distance = 0.32
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
            best_critical = 5

            for pos in av_poses:
                if not (self.boundaries[0] + self.robot_radius < pos[0] < self.boundaries[1] - self.robot_radius and 
                        self.boundaries[2] + self.robot_radius < pos[1] < self.boundaries[3] - self.robot_radius): 
                    continue
                    
                target_pos = (pos[0], pos[1])
                
                path, dist, critical_level = self.get_best_path(target_pos, allow_critical=False)

                if dist < best_dist:
                    best_dist = dist
                    best_path = path
                    best_pos = pos
                    best_critical = critical_level

            penality_cursor = 0
            if z_id == self.zone_cursor_release_id:
                cpc = self.coeff_penalty_cursor * (0.7 * self.match_time - (time.time() - self.start_match_time))
                penality_cursor = - cpc if cpc > 0 else 0

            # Write the final score into the zone
            if best_dist != float('inf'):
                reward = self.compute_release_penality(best_pos[0], best_pos[1], best_dist, best_critical, penality_cursor)
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
                "x": self.boundaries[0],
                "y": self.f_zone_y_min,
                "w": self.boundaries[1],
                "h": self.boundaries[3] - self.f_zone_y_min
            })

            # 3. Store Crate Bubbles (Dynamic)s
            for cid, crate in self.haz_crates.items():
                if crate.state == -1: # Only floor crates
                    self.morbidity_data["crate_bubbles"].append({
                        "id": cid,
                        "x": crate.x,
                        "y": crate.y,
                        "radius": self.classic_margin # Ideal Margin
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
            self.send_raw("PINCE 10 0 0")
            time.sleep(0.1)
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

        list_enn = msg.other_robot_position
        if len(list_enn) == 0:
            self.enemy_pos = None
            return
        
        list_ennemis = []
        for enn in list_enn:
            if self.boundaries[0] + 0.15 < enn.x < self.boundaries[1] - 0.15 and self.boundaries[2] + 0.15 < enn.y < self.boundaries[3] - 0.15:
                list_ennemis.append(enn)
        
        if len(list_ennemis) == 0:
            self.enemy_pos = None
            return

        closer = np.sqrt((list_ennemis[0].x - msg.robot_position.x) ** 2 + (list_ennemis[0].y - msg.robot_position.y) ** 2)
        self.enemy_pos = Position()
        self.enemy_pos.x = list_ennemis[0].x
        self.enemy_pos.y = list_ennemis[0].y
        for pos in list_ennemis:
            dst = np.sqrt((pos.x - msg.robot_position.x) ** 2 + (pos.y - msg.robot_position.y) ** 2)
            if dst < closer:
                self.enemy_pos.x = pos.x
                self.enemy_pos.y = pos.y

    def move_to(self, pos: Position, seuil=0.1, params=None):
        """Compute the move_to action."""
        if not self.stop:
            if self.pos_obj == None:
                self.pos_obj = self.robot_pos
            self.timer = None
            self.seuil = seuil
            self.is_robot_moving = True
            self.is_robot_arrived = False
            time.sleep(0.1)
            if pos.t == None:
                pos.t = self.nearest_great_angle(self.pos_obj.t)
            if params is not None:
                command = f"MOVE {pos.x} {pos.y} {pos.t} {' '.join(params)}"
            else:
                command = f"MOVE {pos.x} {pos.y} {pos.t}"
            self.pub_command.publish(String(data=command))
            self.pos_obj = Position(x=pos.x, y=pos.y, t=pos.t)
            self.motion_done_event.clear()  # Block the wait
            self.motion_done = False
            self.last_time_move = time.time()

    def nearest_great_angle(self, current):
        """Helper to find the nearest angle to 0, pi/2, pi, 3pi/2."""
        angles = [-np.pi/2, -np.pi, -3*np.pi/2, -2*np.pi, 0, np.pi/2, np.pi, 3*np.pi/2, 2*np.pi]
        return min(angles, key=lambda x: abs(current - x))

    def follow_ennemi(self):
        if not self.stop:
            x_middle = 1.5
            y_middle = 1.0
            while True:
                if self.enemy_pos is not None:
                    self.send_raw("VMAX 0.8")
                    v1_x = x_middle - self.enemy_pos.x
                    v1_y = y_middle - self.enemy_pos.y
                    v1_norm = np.sqrt(v1_x ** 2 + v1_y ** 2)
                    v1_x /= v1_norm
                    v1_y /= v1_norm
                    pos_x = self.enemy_pos.x # + v1_x * 0.5
                    pos_y = self.enemy_pos.y # + v1_y * 0.5
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

    def send_plier_cmd(self, ids: dict, crate_dict: dict):
        self.get_logger().info(f"Sending to ids {ids}")
        self.last_time_pliers = time.time()
        if self.stop or not ids:
            return

        self.pliers_done = False
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

    def generate_stacks(self, crate_dict):
        """
        Calculates stacks using Geometric Adjacency and Connected Components.
        Rules: Crates must be parallel, aligned lengthwise, side-by-side (~0.05m apart), max 4 per stack.
        """
        # --- Strict Geometric Tolerances ---
        WIDTH = 0.05  # Ideal distance between centers when side-by-side
        ANGLE_TOLERANCE = math.radians(25)  # 15 degrees max rotation difference
        LONG_TOLERANCE = 0.05  # Allow up to 3cm of sliding/misalignment lengthwise
        LAT_TOLERANCE = 0.05   # Allow the lateral gap to be between 3cm and 7cm
        
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

    def start_state_machine(self):
        """Start the background control loop."""
        if not self._thread.is_alive():
            self._stop_event.clear()
            self._thread.start()

    def stop_state_machine(self):
        """Stop the background control loop."""
        self._stop_event.set()
        if self._thread.is_alive():
            self._thread.join()
        self.global_sm = GlobalSM.NOP

    def _run_loop(self):
        """Run the threaded loop."""
        period = 1.0 / self.loop_frequency
        self.start_match_time = time.time()
        self.send_raw("VMAX 1.5")
        self.send_raw("VTMAX 3.0")
        self.move_to(Position(x=self.entry_zone[0], y=self.entry_zone[1], t=self.robot_pos.t))
        while not self._stop_event.is_set():
            start_local_time = time.time()
            try:
                self.main_loop()
            except Exception as e:
                print(f"🔥 Error in PliersSystem loop: {e}")
                import traceback
                traceback.print_exc()

            # Sleep to maintain frequency
            elapsed = time.time() - start_local_time
            sleep_time = period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                print(f"Warning! Loop duration for pliers took more than expected: {elapsed:.3f}s")

    def main_loop(self):
        """Run main loop logic."""
        if self.break_engage:
            self.break_engage = False
            self.motion_done = True
            self.pliers_done = True
            self.get_logger().info(f"Break engaged")
            self.global_sm = GlobalSM.NOP
            self.sub_sm = GlobalSM.NOP

        if time.time() - self.start_match_time > self.match_time * 3 / 4:
            self.end_far_zone = True
        if time.time() - self.start_match_time > self.match_time - 0.2:
            if self.global_sm not in (GlobalSM.STOP, GlobalSM.FINISH):
                self.get_logger().warn("Stop sequence active! Overriding to STOP.")
                self.global_sm = GlobalSM.STOP
                self.sub_sm = GlobalSM.NOP

        elif time.time() - self.start_match_time > self.match_time - 0.1:
            if self.global_sm not in GlobalSM.GOHOME and not self.arrived_home:
                self.get_logger().warn("Backstage sequence active! Overriding to GOHOME.")
                self.global_sm = GlobalSM.GOHOME
                self.sub_sm = GlobalSM.NOP
                path_e, _, _ = self.get_best_path(self.entry_zone, allow_critical=True)
                self.payload = {"path": path_e}
        
        elif self.global_sm == GlobalSM.NOP:
            if self.cursor_begin_done and not self.cursor_end_done:
                self.cursor_end_done = True
                self.send_raw(f"PINCE {self.pince_cursor} 8 0 0")
            self.send_raw(f"PINCE {self.pince_cursor} 8 0 0")
            req_sm, payload = self.select_strategy()
            
            self.get_logger().info(f"Strategy selected: {req_sm} at {time.time() - self.start_match_time}")
            self.global_sm = req_sm
            self.sub_sm = GlobalSM.NOP
            self.payload = payload

        if self.obstacle_detected and not self.global_sm not in (GlobalSM.STOP, GlobalSM.FINISH):
            self.get_logger().info(f"Obstacle detected by {self.get_namespace()}, enemy at {self.enemy_pos}. Recomputing...")
            self.global_sm = GlobalSM.NOP
            self.sub_sm = GlobalSM.NOP

        if not self.pliers_done and time.time() - self.last_time_pliers > 5.0:
            self.get_logger().info(f"Timeout pliers, restarting by soft")
            self.pliers_done = True

        if not self.motion_done and time.time() - self.last_time_move >  5.0:
            self.get_logger().info(f"Timeout movement, restarting by soft")
            self.motion_done = True

        if (not self.motion_done or not self.pliers_done) and self.global_sm != GlobalSM.STOP: # or time.time() - self.last_enemy_detected < 5.0:
            return
        
        if time.time() - self.start_match_time > 85.0:
            self.send_raw("VMAX 0.3")
            self.release_end = True

        match self.global_sm:
            case GlobalSM.NOP:
                pass
            case GlobalSM.FINISH:
                pass
            case GlobalSM.EXPLORE:
                self.explore_sm()
            case GlobalSM.PICK:
                self.pick_sm()
            case GlobalSM.RELEASE:
                self.release_sm()
            case GlobalSM.GOHOME:
                self.go_home_sm()
            case GlobalSM.STOP:
                self.stop_sm()
            case GlobalSM.CURSOR:
                self.cursor_sm()
            case GlobalSM.GOBOARD:
                self.go_board_sm()
            case _:
                print(f"Unknown State: {self.global_sm}")

    # =========================================================================
    # SELECT STRATEGY
    # =========================================================================
    def select_strategy(self):
        """Plan all the options and their rewards."""

        if self.activate_cursor and not self.cursor_end_done:
            self.cursor_begin_done = True
            self.get_logger().info(f"CURSOR DONE")
            return GlobalSM.CURSOR, {}

        self.compute_and_update_rewards()
        
        with self.data_lock:
            best_zone = max(self.zones.values(), key=lambda z: z.release_reward, default=None)
            best_crate = max(self.haz_crates.values(), key=lambda c: c.pick_reward, default=None)
            # self.get_logger().info(f"Best create : {best_crate}, Best zone: {best_zone}")

            pick_available = best_crate and best_crate.pick_reward > -100
            release_available = best_zone and best_zone.release_reward > -100

            if pick_available and release_available:
                if best_crate.pick_reward > best_zone.release_reward:
                    pick_dict = {
                        "crate_ids": best_crate.is_part_of_stack,
                        "path": best_crate.best_pick_path[1:], 
                        "inv": best_crate.use_inverted
                    }
                    self.get_logger().info(f"Select PICK COMP with: {pick_dict}")
                    return GlobalSM.PICK, pick_dict
                
                else:
                    release_dict = {
                        "path": best_zone.best_release_path[1:], 
                        "best_pos": best_zone.best_release_pos,
                        "id_zone": best_zone.id,
                    }
                    self.get_logger().info(f"Select RELEASE COMP with: {release_dict}")
                    return GlobalSM.RELEASE, release_dict
                
            elif pick_available:
                pick_dict = {
                    "crate_ids": best_crate.is_part_of_stack, 
                    "path": best_crate.best_pick_path[1:],
                    "inv": best_crate.use_inverted
                }
                self.get_logger().info(f"Select PICK with: {pick_dict}")
                return GlobalSM.PICK, pick_dict
            
            elif release_available:
                release_dict = {
                    "path": best_zone.best_release_path[1:], 
                    "best_pos": best_zone.best_release_pos,
                    "id_zone": best_zone.id,
                }
                self.get_logger().info(f"Select RELEASE with: {release_dict}")
                return GlobalSM.RELEASE, release_dict   
            
            # Initialize id_steal if it doesn't exist yet
            if not hasattr(self, 'id_steal'): self.id_steal = 0

            pos = self.steal_poses[self.id_steal % len(self.steal_poses)]
            path, dst_test, critic_level = self.get_best_path((pos[0], pos[1]), allow_critical=True)
            best_path = path
            attempts = 0
            dst = 10000
            while attempts < len(self.steal_poses) - 1 and critic_level > 2:
                pos = self.steal_poses[self.id_steal % len(self.steal_poses)]
                path, dst_test, critic_level = self.get_best_path((pos[0], pos[1]), allow_critical=True)
                if dst_test < dst:
                    dst = dst_test
                    best_path = path.copy()
                attempts += 1
                self.id_steal += 1

            if dst_test > 100:
                if not self.is_point_safe(self.robot_pos.x, self.robot_pos.y):
                    return GlobalSM.GOBOARD, {}
                return GlobalSM.NOP, {}
            return GlobalSM.EXPLORE, {"path": best_path}

    # =========================================================================
    # PICK STATE MACHINE (Updated with dynamic camera logic & retries)
    # =========================================================================
    def pick_sm(self):
        match self.sub_sm:
            case GlobalSM.NOP:
                if len(self.payload['path']) != 0:
                    self.get_logger().info(f"Initiating Pick Sequence at {self.payload['path'][-1]}...")
                    available_sides = self.get_best_side_pliers(len(self.payload["crate_ids"]))
                
                    # OPTIMISATION : Trouver la face qui nécessite le moins de rotation
                    with self.data_lock:
                        target_pos = self.get_mean_pose([self.haz_crates[pid] for pid in self.payload["crate_ids"]])
                        self.payload["close_cursor"] = self.in_cursor_zone(target_pos.x, target_pos.y)

                        best_side_ids = available_sides[0]
                        min_rotation = float('inf')
                        for side_pliers in available_sides:
                            # On prend l'angle de la première pince de ce côté
                            side_theta = self.pliers[side_pliers[0]].theta
                            # On calcule combien le robot devrait tourner pour aligner cette face
                            target_angle = target_pos.t + (np.pi if self.payload["inv"] else 0.0)
                            rotation_needed = self.angular_distance(self.robot_pos.t + side_theta, target_angle)
                            
                            if rotation_needed < min_rotation:
                                min_rotation = rotation_needed
                                best_side_ids = side_pliers

                    selected_pliers_ids = best_side_ids
                    self.payload["selected_pliers_ids"] = selected_pliers_ids
                    with self.data_lock:
                        target_pos = self.get_mean_pose([self.haz_crates[pid] for pid in self.payload["crate_ids"]])
                        pliers_pos = self.get_mean_pose([self.pliers[pid] for pid in selected_pliers_ids])
                        
                    target_angle = target_pos.t + (np.pi if self.payload["inv"] else 0.0)
                    self.payload["final_robot_theta"] = target_angle - pliers_pos.t
                else: 
                    self.get_logger().info(f"Initiating Pick Sequence with an unknown path...")
                self.sub_sm = PickSM.PICK_NAV
                self.payload["retries"] = 0
            
            case PickSM.PICK_NAV:
                self.get_logger().info(f"Payload nav {self.payload['path']}...")
                if not self.payload["path"]:
                    self.sub_sm = PickSM.PICK_STARE
                else:
                    next_point = self.payload["path"].pop(0)
                    self.move_to(Position(x=next_point[0], y=next_point[1], t=self.payload["final_robot_theta"]))

            case PickSM.PICK_STARE:
                if self.count > int(self.time_stare * self.loop_frequency):
                    self.sub_sm = PickSM.PICK_UPDATE
                    self.count = 0
                else:
                    if self.count == 0:
                        self.get_logger().debug(f"Staring for a sec")
                        self.payload["stare_pos"] = Position(x=self.robot_pos.x, y=self.robot_pos.y, t=self.robot_pos.t)
                    self.count += 1

            case PickSM.PICK_UPDATE:
                # 1. Identify which camera is facing the stack
                self.activate_cursor = not self.cursor_end_done and self.payload["close_cursor"]

                with self.data_lock:
                    pliers_pos = self.get_mean_pose([self.pliers[pid] for pid in self.payload["selected_pliers_ids"]])
                
                id_side = None
                for cam in self.cameras.values():
                    if are_angles_close(pliers_pos.t, cam.angle, tolerance=0.2):
                        id_side = cam.id
                        break
                
                self.get_logger().debug(f"Using camera {id_side} for final confirmation.")
                
                # 2. Stare & get a snapshot specifically for this camera
                target_view_crates = self.stare_and_update(self.haz_crates, camera_target=[id_side])

                if id_side is not None:
                    temp_dict = {c.id: c for c in target_view_crates}
                    fresh_stacks = self.generate_stacks(temp_dict)

                    if not fresh_stacks:
                        self.get_logger().debug("Stack vanished or broke geometry. Aborting pick.")
                        self.sub_sm = PickSM.PICK_FAILED
                        return

                    with self.data_lock:
                        original_ids = self.payload.get("crate_ids", [])
                        original_crates = [
                            self.haz_crates[cid] for cid in original_ids
                            if cid in self.haz_crates
                        ]

                    if original_crates:
                        # Centroïde du stack original planifié
                        target_cx = np.mean([c.x for c in original_crates])
                        target_cy = np.mean([c.y for c in original_crates])

                        # Fonction locale pour calculer la distance d'un stack
                        def calculate_stack_distance(stack):
                            return math.hypot(
                                np.mean([temp_dict[sid].x for sid in stack]) - target_cx,
                                np.mean([temp_dict[sid].y for sid in stack]) - target_cy,
                            )

                        # Choisir le stack frais dont le centroïde est le plus proche
                        best_stack_ids = min(fresh_stacks, key=calculate_stack_distance)
                        self.get_logger().info(f"Distance to stack found: {calculate_stack_distance(best_stack_ids)}")

                        # Vérifier si la distance du meilleur stack est > 0.5m
                        if calculate_stack_distance(best_stack_ids) > 0.2:
                            self.get_logger().debug("Stack moved too far (>0.5m). Aborting pick.")
                            self.sub_sm = PickSM.PICK_FAILED
                            return

                    else:
                        # Fallback : plus d'infos sur la cible, on prend le premier
                        self.get_logger().warn("Cibles originales introuvables. Fallback sur fresh_stacks[0].")
                        best_stack_ids = fresh_stacks[0] 
                    
                    with self.data_lock:
                        actual_stack_crates = [self.haz_crates[tid] for tid in best_stack_ids]
                        target_pos = self.get_mean_pose(actual_stack_crates)
                        target_pos.t = mean_angle_mod_pi([crate.theta for crate in actual_stack_crates])

                        # Shift pliers configuration to match the new size of the stack
                        sel_pl_ids = self.payload["selected_pliers_ids"]
                        new_pliers_selected = list(range(sel_pl_ids[0], sel_pl_ids[0] + len(best_stack_ids)))
                        pliers_pos = self.get_mean_pose([self.pliers[pid] for pid in new_pliers_selected])
                        self.payload["final_pick_crate_ids"] = best_stack_ids
                        self.payload["final_selected_pliers_ids"] = new_pliers_selected

                    # 5. RE-CALCULATE Final Alignment dynamically
                    x_off, y_off = self.rotate_point(pliers_pos.x, pliers_pos.y, target_pos.t - pliers_pos.t)
                    final_pos = Position(target_pos.x - x_off, target_pos.y - y_off, target_pos.t - pliers_pos.t)
                    dst = (self.robot_pos.x - final_pos.x) ** 2 + (self.robot_pos.y - final_pos.y) ** 2
                    x_off2, y_off2 = self.rotate_point(pliers_pos.x, pliers_pos.y, target_pos.t - pliers_pos.t + np.pi)
                    final_pos_2 = Position(target_pos.x - x_off2, target_pos.y - y_off2, target_pos.t - pliers_pos.t + np.pi)
                    inv = False
                    if (self.robot_pos.x - final_pos_2.x) ** 2 + (self.robot_pos.y - final_pos_2.y) ** 2 < dst:
                        final_pos = final_pos_2
                        inv = True
                    self.payload["inv"] = inv

                self.payload["final_pos"] = final_pos
                self.payload["final_path"], _, _ = self.get_best_path((final_pos.x, final_pos.y), allow_critical=True)
                
                self.sub_sm = PickSM.PICK_CENTERING

            case PickSM.PICK_CENTERING:
                if not self.payload["final_path"]:
                    self.sub_sm = PickSM.PICK_PICK
                else:
                    with self.data_lock:
                        target_pos = self.get_mean_pose([self.haz_crates[pid] for pid in self.payload["final_pick_crate_ids"]])
                        pliers_pos = self.get_mean_pose([self.pliers[pid] for pid in self.payload["final_selected_pliers_ids"]])
                    target_angle = mean_angle_mod_pi([self.haz_crates[pid].theta for pid in self.payload["final_pick_crate_ids"]])
                    final_robot_theta = target_angle - pliers_pos.t + (np.pi if self.payload["inv"] else 0.0)

                    next_point = self.payload["final_path"].pop(0)
                    self.move_to(Position(x=next_point[0], y=next_point[1], t=final_robot_theta))

            case PickSM.PICK_PICK:
                final_pos = self.payload["final_pos"]
                selected_pliers = self.payload["final_selected_pliers_ids"]
                pick_targets = self.payload["final_pick_crate_ids"]

                with self.data_lock:
                    robot_x, robot_y, robot_t = final_pos.x, final_pos.y, final_pos.t
                    pliers_in_map = {}
                    
                    # Project pliers into world space
                    for pl_id in selected_pliers:
                        p_local = self.pliers[pl_id]
                        rx, ry = self.rotate_point(p_local.x, p_local.y, robot_t)
                        pliers_in_map[pl_id] = (robot_x + rx, robot_y + ry)

                    remaining_targets = [tid for tid in pick_targets if tid in self.haz_crates]
                    
                    if not remaining_targets:
                        self.get_logger().debug("All targets vanished during approach! Aborting.")
                        self.sub_sm = PickSM.PICK_FAILED
                        return 

                    # Hungarian Assignment for optimal grip mapping
                    plier_list = list(pliers_in_map.keys())
                    target_list = list(remaining_targets)
                    cost_matrix = np.zeros((len(plier_list), len(target_list)))
                    
                    for i, pl_id in enumerate(plier_list):
                        px, py = pliers_in_map[pl_id]
                        for j, obj_id in enumerate(target_list):
                            cx, cy = self.haz_crates[obj_id].x, self.haz_crates[obj_id].y
                            cost_matrix[i, j] = (cx - px)**2 + (cy - py)**2

                    row_ind, col_ind = linear_sum_assignment(cost_matrix)

                    dict_sel_pliers = {}
                    for i, j in zip(row_ind, col_ind):
                        pl_id, best_obj_id = plier_list[i], target_list[j]
                        dict_sel_pliers[pl_id] = ["pick", best_obj_id]

                self.send_plier_cmd(dict_sel_pliers, self.haz_crates)
                self.sub_sm = PickSM.PICK_DONE

            case PickSM.PICK_DONE:
                # 1. On compte combien de pinces ont réussi
                selected_pliers = self.payload.get("final_selected_pliers_ids", [])
                picked_count = sum(1 for pid in selected_pliers if self.pliers[pid].state != -1)

                # 2. Si échec total et qu'on n'a pas encore réessayé
                if picked_count == 0 and self.payload.get("retries", 0) < 1:
                    self.get_logger().warn("Pick completely failed! Attempting one retry...")
                    self.sub_sm = PickSM.PICK_RETRY
                else:
                    # Succès (ou on a déjà réessayé), on libère la machine à état
                    if picked_count == 0:
                        self.get_logger().error("Pick completely failed after retry.")
                    self.global_sm = GlobalSM.NOP
                    self.sub_sm = GlobalSM.NOP

            case PickSM.PICK_RETRY:
                self.payload["retries"] += 1

                # 1. Identifier les pinces utilisées pour déterminer la face active
                sel_pl_ids = self.payload.get("final_selected_pliers_ids", [])
                if not sel_pl_ids:
                    sel_pl_ids = self.payload.get("selected_pliers_ids", [])

                # 2. Obtenir l'angle local (sur le robot) de ces pinces
                pliers_pos = self.get_mean_pose([self.pliers[pid] for pid in sel_pl_ids])
                
                # 3. Calculer l'angle global de la face (orientation du robot + orientation de la face)
                face_global_theta = self.robot_pos.t + pliers_pos.t
                
                # 4. Calculer le point de recul : on retranche 10cm (0.1m) sur l'axe de la face
                new_x = self.robot_pos.x - 0.1 * math.cos(face_global_theta)
                new_y = self.robot_pos.y - 0.1 * math.sin(face_global_theta)
                
                # 5. Déplacer le robot tout en gardant son orientation angulaire intacte
                self.move_to(Position(x=new_x, y=new_y, t=self.robot_pos.t))
                # On reboucle sur l'étape de prise de vue
                self.sub_sm = PickSM.PICK_STARE

            case PickSM.PICK_FAILED:
                self.global_sm = GlobalSM.NOP
                self.sub_sm = GlobalSM.NOP

    # =========================================================================
    # RELEASE STATE MACHINE (Converted to pure SM architecture)
    # =========================================================================
    def release_sm(self):
        match self.sub_sm:
            case GlobalSM.NOP:
                self.sub_sm = ReleaseSM.RELEASE_NAV
                
            case ReleaseSM.RELEASE_NAV:
                if not self.payload["path"]:
                    self.sub_sm = ReleaseSM.RELEASE_DROP
                else:
                    best_pos = self.payload["best_pos"]
                    with self.data_lock:
                        id_side = self.get_best_side_pliers_release(best_pos[2])
                        pliers_theta = self.pliers[id_side * 4].theta if id_side is not None else 0.0
                    
                    self.payload["id_side"] = id_side
                    target_angle = best_pos[2] - pliers_theta
                    
                    next_point = self.payload["path"].pop(0)
                    self.move_to(Position(x=next_point[0], y=next_point[1], t=target_angle))

            case ReleaseSM.RELEASE_DROP:
                id_side = self.payload.get("id_side")
                
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
                    
                    # Clear holding state natively
                    for pl_id in id_active_pliers:
                        self.pliers[pl_id].state = -1

                self.send_plier_cmd(id_active_pliers, self.haz_crates)
                self.sub_sm = ReleaseSM.RELEASE_DONE

            case ReleaseSM.RELEASE_DONE:
                if self.payload["id_zone"] == self.zone_cursor_release_id:
                    self.cursor_end_done = True
                self.global_sm = GlobalSM.NOP
                self.sub_sm = GlobalSM.NOP

    # =========================================================================
    # CURSOR STATE MACHINE (Converted to pure SM architecture)
    # =========================================================================
    def cursor_sm(self):
        match self.sub_sm:
            case GlobalSM.NOP:
                self.sub_sm = CursorSM.CURSOR_NAV
                self.payload['path'], _, _ = self.get_best_path((self.cursor_in[0], self.cursor_in[1] + 0.3), allow_critical=True)
                
            case CursorSM.CURSOR_NAV:
                if not self.payload["path"]:
                    self.sub_sm = CursorSM.CURSOR_APPROACH_DOWN
                else:
                    next_point = self.payload["path"].pop(0)
                    self.move_to(Position(x=next_point[0], y=next_point[1], t=self.cursor_in[2]))

            case CursorSM.CURSOR_APPROACH_DOWN:
                self.send_raw(f"PINCE {self.pince_cursor} 7 0 0")
                self.pliers_done = False
                self.move_to(Position(x=self.cursor_in[0], y=self.cursor_in[1], t=self.cursor_in[2]))
                self.sub_sm = CursorSM.CURSOR_CURSOR

            case CursorSM.CURSOR_CURSOR:
                self.move_to(Position(x=self.cursor_out[0], y=self.cursor_out[1], t=self.cursor_out[2]))
                self.sub_sm = CursorSM.CURSOR_LEAVE_UP
            
            case CursorSM.CURSOR_LEAVE_UP:
                self.get_logger().info(f"LAst")
                self.send_raw(f"PINCE {self.pince_cursor} 8 0 0")
                self.pliers_done = False
                self.move_to(Position(x=self.cursor_out[0], y=self.cursor_out[1] + 0.3, t=self.cursor_out[2]))
                self.sub_sm = CursorSM.CURSOR_DONE

            case CursorSM.CURSOR_DONE:
                self.cursor_end_done = True
                self.global_sm = GlobalSM.NOP
                self.sub_sm = GlobalSM.NOP

    # =========================================================================
    # EXPLORE STATE MACHINE
    # =========================================================================
    def explore_sm(self):
        match self.sub_sm:
            case GlobalSM.NOP:
                self.get_logger().info("Nothing to do: Exploring / Stealing...")
                self.sub_sm = ExploreSM.EXPLORE_NAV
                self.payload["req_stare"] = False
            
            case ExploreSM.EXPLORE_NAV:
                if self.payload["req_stare"]:
                    self.sub_sm = ExploreSM.EXPLORE_STARE
                else:
                    next_point = self.payload["path"].pop(0)
                    self.move_to(Position(x=next_point[0], y=next_point[1])) #, t=(self.id_steal * np.pi) % (2 * np.pi)
                    self.payload["req_stare"] = True

            case ExploreSM.EXPLORE_STARE:
                if self.count > int(self.time_stare * self.loop_frequency):
                    self.sub_sm = ExploreSM.EXPLORE_UPDATE
                    self.count = 0
                else:
                    if self.count == 0:
                        self.get_logger().debug(f"Staring for a sec")
                    self.count += 1

            case ExploreSM.EXPLORE_UPDATE:
                self.stare_and_update(self.haz_crates, camera_target=[1, 2, 3])
                self.payload["req_stare"] = False
                if not self.payload["path"]:
                    self.sub_sm = ExploreSM.EXPLORE_DONE
                else:
                    self.sub_sm = ExploreSM.EXPLORE_NAV

            case ExploreSM.EXPLORE_DONE:
                self.id_steal += 1
                self.global_sm = GlobalSM.NOP
                self.sub_sm = GlobalSM.NOP

    # =========================================================================
    # GO HOME STATE MACHINE
    # =========================================================================
    def go_home_sm(self):
        match self.sub_sm:
            case GlobalSM.NOP:
                self.sub_sm = GoHomeSM.GOHOME_NAV_E_ZONE

            case GoHomeSM.GOHOME_NAV_E_ZONE:
                if not self.payload["path"]:
                    self.sub_sm = GoHomeSM.GOHOME_NAV_FINAL
                    self.payload["final_path"], _, _ = self.get_best_path((self.final_zone["x"], self.final_zone["y"]), allow_critical=True)
                else:
                    next_point = self.payload["path"].pop(0)
                    self.move_to(Position(x=next_point[0], y=next_point[1]))

            case GoHomeSM.GOHOME_NAV_FINAL:
                if not self.payload["final_path"]:
                    self.sub_sm = GoHomeSM.GOHOME_DROP
                else:
                    next_point = self.payload["final_path"].pop(0)
                    self.move_to(Position(x=next_point[0], y=next_point[1]))

            case GoHomeSM.GOHOME_DROP:
                self.get_logger().info("Dropping all crates.")
                with self.data_lock:
                    self.send_raw("PINCE 10 0 0")
                    for plier in self.pliers.values():
                        if plier.state == -1:
                            continue
                        
                        crate = self.haz_crates[plier.state]
                        plier.state = -1
                        plier.is_running = True
                        crate.state = -1
                        self.update_crate_pos(plier, crate)
                
                self.sub_sm = GoHomeSM.GOHOME_DONE
                
            case GoHomeSM.GOHOME_DONE:
                self.global_sm = GlobalSM.NOP
                self.sub_sm = GlobalSM.NOP
                self.arrived_home = True

    def go_board_sm(self):
        match self.sub_sm:
            case GlobalSM.NOP:
                if self.robot_pos.x < self.safe_x_min + 0.05:
                    self.payload["xpos"] = self.safe_x_min + 0.15

                elif self.robot_pos.x > self.safe_x_max - 0.05:
                    self.payload["xpos"] = self.safe_x_max - 0.15

                else:
                    self.payload["xpos"] = self.robot_pos.x

                if self.robot_pos.y < self.safe_y_min + 0.05:
                    self.payload["ypos"] = self.safe_y_min + 0.15

                elif self.robot_pos.y > self.f_zone_y_min - 0.05:
                    self.payload["ypos"] = self.f_zone_y_min - 0.15
                
                else:
                    self.payload["ypos"] = self.robot_pos.y
                self.sub_sm = GoBoardSM.GO_BOARD_NAV

            case GoBoardSM.GO_BOARD_NAV:
                self.move_to(Position(x=self.payload["xpos"], y=self.payload["ypos"]))
                self.sub_sm = GoBoardSM.GO_BOARD_DONE

            case GoBoardSM.GO_BOARD_DONE:
                self.global_sm = GlobalSM.NOP
                self.sub_sm = GlobalSM.NOP

    def stop_sm(self):
        match self.sub_sm:
            case GlobalSM.NOP:
                self.sub_sm = StopSM.STOP_STOP

            case StopSM.STOP_STOP:
                self.send_raw("BLOCK")
                self.send_raw("PINCE 10 0 0")

            case StopSM.STOP_DONE:
                self.global_sm = GlobalSM.FINISH
                self.sub_sm = GlobalSM.NOP

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
    
    def in_cursor_zone(self, x, y):
        """
        Checks if ANY part of a rotated crate (0.15m x 0.05m) is inside the zone.
        Uses the Separating Axis Theorem (SAT) for OBB-AABB collision.
        """
        zx = self.zone_cursor_pick[0]
        zy = self.zone_cursor_pick[1]
        hz = 0.125  # Zone half-size
        
        # --- Axis 1 & 2: Global X and Y (The Zone's flat edges) ---
        if abs(x - zx) > hz:
            return False # Shadows don't overlap on X, skip to next zone!
            
        if abs(y - zy) > hz:
            return False # Shadows don't overlap on Y, skip to next zone!
            
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
                if c.state != -1:
                    continue
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

    def compute_release_penality(self, px, py, path_distance, critical_level, penalty_cursor):
        """Calculates the score of a specific release pose using actual path distance."""
        
        val_center = (1.5 - px) ** 2 + (1 - py) ** 2
        val_enn_zone = abs(self.boundaries[self.color] - px) * int(self.end_far_zone)
        # Use the TRUE path distance, not just a straight line guess!
        val_dst = path_distance**2 
        
        if self.enemy_pos is not None:
            val_ennemi = (self.enemy_pos.x - px) ** 2 + (self.enemy_pos.y - py) ** 2
        else:
            val_ennemi = 0
            
        return (
            self.coeff_release + 
            self.coeff_release_dst * val_dst + 
            self.coeff_release_enn * val_ennemi + 
            self.coeff_release_center * val_center + 
            self.coeff_critical_level * critical_level +
            self.coeff_far_zone * val_enn_zone +
            penalty_cursor
        )

    def compute_pick_penality(self, x, y, num_crates, path_distance, critical_level, balance):
        val_center = (1.5 - x) ** 2 + (1 - y) ** 2
        val_enn_zone = abs(self.boundaries[self.color] - x) * int(self.end_far_zone)
        val_dst = path_distance ** 2
        if self.enemy_pos is not None:
            val_ennemi = (self.enemy_pos.x - x) ** 2 + (self.enemy_pos.y - y) ** 2
        else:
            val_ennemi = 0
        return (
            self.coeff_pick +
            self.coeff_pick_balance * abs(balance) + 
            self.coeff_pick_dst * val_dst + 
            self.coeff_pick_enn * val_ennemi + 
            self.coeff_pick_center * val_center + 
            self.coeff_pick_num * num_crates + 
            self.coeff_critical_level * critical_level +
            self.coeff_far_zone * val_enn_zone
        )

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
            if side == 2:
                continue
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

    def get_street_grid_path(self, start, target, margin):
        import heapq
        
        def pt(x, y): return (round(x, 4), round(y, 4))
        
        start = pt(*start)
        target = pt(*target)
        nodes = set([start, target])
        
        # --- NEW: Use safe boundaries for the street grid ---
        x_lines = [self.safe_x_min, self.safe_x_max, start[0], target[0]]
        y_lines = [self.safe_y_min + 0.2, self.f_zone_y_min, start[1], target[1]]
        
        # --- NEW: Add dynamic grid lines around the enemy to route around it ---
        if getattr(self, 'enemy_pos', None) is not None:
            x_lines.extend([self.enemy_pos.x - 2 * self.robot_radius, self.enemy_pos.x + 2 * self.robot_radius])
            y_lines.extend([self.enemy_pos.y - 2 * self.robot_radius, self.enemy_pos.y + 2 * self.robot_radius])
        
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
                # Verify the edge doesn't cut through the forbidden zone OR the enemy
                if self.is_direct_path_clear(n1, n2, margin=margin, target_crate_ids=None):
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

    def is_direct_path_clear(self, start_pos, target_pos, margin, target_crate_ids=None):
        """Checks if the straight line hits crates, the forbidden zone, OR the enemy."""
        x1, y1 = start_pos
        x2, y2 = target_pos
        
        target_crate_ids = None
        # Normalize target_crate_ids to always be a list
        if target_crate_ids is None:
            target_crate_ids = []
        elif not isinstance(target_crate_ids, list):
            target_crate_ids = [target_crate_ids]

        # --- Quick check if start or end are fundamentally unsafe ---
        if not self.is_point_safe(x1, y1) or not self.is_point_safe(x2, y2):
            return False

        dx = x2 - x1
        dy = y2 - y1
        length = math.hypot(dx, dy)
        
        if length == 0:
            return True
            
        # --- Sample points along the line to check for forbidden zone intersection ---
        steps = int(length / 0.1)
        if steps > 0:
            for i in range(1, steps):
                tx = x1 + (dx * (i / steps))
                ty = y1 + (dy * (i / steps))
                if not self.is_point_safe(tx, ty):
                    return False # The path cuts through the forbidden zone!

        length_sq = length**2

        # --- Check against Enemy Robot ---
        if self.enemy_pos is not None:
            # Calculate RAW t without bounding it to 0.0
            t_raw = ((self.enemy_pos.x - x1) * dx + (self.enemy_pos.y - y1) * dy) / length_sq
            
            # If t_raw < 0, it's behind us. Only check if it's >= 0
            if t_raw >= 0.0:
                closest_x = x1 + t_raw * dx
                closest_y = y1 + t_raw * dy
                
                if math.hypot(self.enemy_pos.x - closest_x, self.enemy_pos.y - closest_y) < 3 * self.robot_radius:
                    return False

        # --- Check against crates ---
        for cid, crate in self.haz_crates.items():
            if cid in target_crate_ids or crate.state != -1:
                continue

            # Calculate RAW t
            t_raw = ((crate.x - x1) * dx + (crate.y - y1) * dy) / length_sq
            
            if t_raw < 0.0 or t_raw > 1.0:
                continue # The crate is mathematically behind us, ignore it!

            closest_x = x1 + t_raw * dx
            closest_y = y1 + t_raw * dy
            
            if math.hypot(crate.x - closest_x, crate.y - closest_y) < margin:
                return False
                 
        return True
    
    def get_best_path(self, target_pos, target_crate_ids=None, allow_critical=False):
        """
        - If allow_critical=False: Returns path ONLY if 0.35m margin is respected.
        - If allow_critical=True: Tries 0.35m, then 0.23m, then returns direct line 
        no matter what (Best Effort).
        """
        start_pos = (self.robot_pos.x, self.robot_pos.y)

        # --- 1. TRY THE SAFE WAY ---
        if self.is_direct_path_clear(start_pos, target_pos, margin=self.classic_margin, target_crate_ids=target_crate_ids):
            return [start_pos, target_pos], math.hypot(target_pos[0]-start_pos[0], target_pos[1]-start_pos[1]), 0

        path, cost = self.get_street_grid_path(start_pos, target_pos, margin=self.classic_margin)
        if path:
            return path, cost, 1

        # --- 2. IF NOT ALLOWED TO BE RISKY, STOP HERE ---

        # --- 3. ALLOWED TO BE RISKY: TRY CRITICAL ---
        self.get_logger().debug("Strict path blocked. Attempting Critical Margin.")

        if self.is_direct_path_clear(start_pos, target_pos, margin=self.critical_margin, target_crate_ids=target_crate_ids):
            return [start_pos, target_pos], math.hypot(target_pos[0]-start_pos[0], target_pos[1]-start_pos[1]), 2

        path, cost = self.get_street_grid_path(start_pos, target_pos, margin=self.critical_margin)
        if path:
            return path, cost, 2
        
        if not allow_critical:
            return [], float('inf'), 3

        path, cost = self.get_street_grid_path(start_pos, target_pos, margin=0)
        if path:
            return path, cost, 3

        # --- 4. ABSOLUTE FALLBACK ---
        return [], float('inf'), 4

def mean_angle_mod_pi(angles_in_radians):
    """
    Calculates the mean of angles modulo pi without trigonometric functions.
    Extremely fast. Returns an angle in the range [0, pi).
    """
    angles = np.asarray(angles_in_radians)
    if len(angles) == 0:
        return 0.0
        
    # Step 1: Pick the first angle as our anchor/reference point
    ref = angles[0]
    
    # Step 2: Find the shortest angular distance from the reference
    # Adding pi/2, doing modulo pi, and subtracting pi/2 forces 
    # all differences to wrap cleanly into the range [-pi/2, +pi/2]
    diffs = (angles - ref + np.pi / 2.0) % np.pi - (np.pi / 2.0)
    
    # Step 3: Average those differences
    mean_diff = np.mean(diffs)
    
    # Step 4: Add the average difference back to the reference and normalize
    return (ref + mean_diff) % np.pi

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