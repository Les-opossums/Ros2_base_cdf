#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Action Sequencer Node."""

### IDEA SMART SCRPIT: 
# Use stacks and also save solo crates so we can choose the strategy. 
# Use the Actuators ID, so we can manage them 1 by one.
# Actions for pliers
# 0: free, 1: picking, 2: dropping, 3: reverting


## TODO: PICK ONLY THE ONES NOT THE ROGHT COLOR IN THE ZONES AND TURN IT. 
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
        self.data_lock = threading.Lock()
        # self.camera_angles = [0.0, 2.093333, 4.186666]
        self.camera_angles = [0.0]

    def _init_mapping(self):
        objects_path = os.path.join(
            get_package_share_directory("opossum_bringup"), "config", str(self.year), f"{self.board_config}.yaml"
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
            self.zones[id] = ZoneRelease(id, zone['x'], zone['y'], zone['size'])
        
        self.final_zone = data['final_zone']

        # Process objects with cam
        self.barycentre = None
        self.centering = False
        self.list_objects = []
        self.is_center = False
        self.try_center = 0
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
        time.sleep(0.5) 
        
        if time.time() - self.last_camera_timestamp > 0.4:
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
                self.get_logger().info(f"Color val: {color_val}, det_id: {det.id} at {det.x}, {det.y}")
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
                self.get_logger().info(f"Color val: {color_val}, det_id: {det.id} at {det.x}, {det.y}")
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
        for cid in ghost_ids:
            self.get_logger().warn(f"GHOST DETECTED: Crate {cid} missing from expected FOV! Erasing.")
            ghost_crate = self.haz_crates[cid]
            
            if ghost_crate.state != -1 and ghost_crate.state in self.pliers:
                self.pliers[ghost_crate.state].state = -1
                    
            for stack_list in self.stacks.values():
                if cid in stack_list:
                    stack_list.remove(cid)
                    
            del self.haz_crates[cid]
        # self._rebuild_stacks()
        self.get_logger().info("Finished staring go back to match.")

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
            p0 = int(data[2])
            p1 = int(data[3])
            if self.pliers[id * 2].is_running:
                self.pliers[id * 2].is_running = False
                if not p0:
                    self.get_logger().warn(f"The first plier of ID {id} failed.")
            if self.pliers[id * 2 + 1].is_running:
                self.pliers[id * 2 + 1].is_running = False
                if not p1:
                    self.get_logger().warn(f"The second plier of ID {id} failed.")

            if not any(pl.is_running for pl in self.pliers.values()):
                self.pliers_event.set()

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
                    self.pub_command.publish(String(data=f"PINCE {cmd_id} 4 2"))
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
                    self.pub_command.publish(String(data=f"PINCE {cmd_id} 5 2"))
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

    def any_plier_used(self):
        return any(pl.state != -1 for pl in self.pliers.values())

    def _rebuild_stacks(self):
        """Dynamically groups adjacent, aligned crates into stacks based on real-world geometry."""
        with self.data_lock:
            self.stacks.clear()
            
            # 1. Only consider crates sitting freely on the floor
            free_crates = [cid for cid, crate in self.haz_crates.items() if crate.state == -1]
            visited = set()
            stack_counter = 0
            
            for cid in free_crates:
                if cid in visited:
                    continue
                    
                current_stack = [cid]
                visited.add(cid)
                
                # 2. Breadth-First Search to find all connected crates in a line
                queue = [cid]
                while queue:
                    curr_id = queue.pop(0)
                    C = self.haz_crates[curr_id]
                    
                    for other_id in free_crates:
                        if other_id in visited:
                            continue
                            
                        O = self.haz_crates[other_id]
                        dx = O.x - C.x
                        dy = O.y - C.y
                        dist = math.hypot(dx, dy)
                        
                        # If they are too far apart to be neighbors, ignore immediately
                        if dist > 0.25:
                            continue
                            
                        # 3. Check Angle Alignment (modulo pi in case one is spun 180 degrees)
                        angle_diff = abs(C.theta - O.theta) % math.pi
                        if angle_diff > math.pi / 2: 
                            angle_diff = math.pi - angle_diff
                            
                        if angle_diff > 0.35: # ~20 degrees tolerance
                            continue
                            
                        # 4. The Magic Math: Perpendicular and Parallel Distances
                        parallel_dist = abs(dx * math.cos(C.theta) + dy * math.sin(C.theta))
                        perp_dist = abs(-dx * math.sin(C.theta) + dy * math.cos(C.theta))
                        
                        # If they are perfectly side-by-side!
                        if parallel_dist < 0.12 and perp_dist < 0.25:
                            current_stack.append(other_id)
                            visited.add(other_id)
                            queue.append(other_id)
                            
                # 5. Only save it as a "stack" if there are at least 2 crates
                if len(current_stack) > 1:
                    self.stacks[stack_counter] = current_stack
                    stack_counter += 1
                    self.get_logger().info(f"Dynamically formed new stack {stack_counter-1} with crates: {current_stack}")

    def smart_moves(self):
        self.send_raw(f"VMAX 0.3")
        self.send_raw(f"VTMAX 1.5")
        
        activate_check_stack = True
        steal_poses = [[0.45, 0.45], [0.45, 1.1], [2.55, 1.1], [2.55, 0.45]]
        id_steal = 0

        while not self.match_finished:
            
            # =================================================================
            # 1. READ WORLD STATE SAFELY (Using Thread Lock)
            # =================================================================
            with self.data_lock:
                pick_crate_ind, stack_id, is_inv, pick_path = self.compute_pick_rewards()
                release_zone_ind, best_release_pos = self.compute_release_rewards()
                pliers_used = any(pl.state != -1 for pl in self.pliers.values())
                start_pos = (self.robot_pos.x, self.robot_pos.y)
                x_min, x_max, y_min, y_max = self.rect

            # =================================================================
            # 2. GO FOR A RELEASE (If we hold crates or decided to release)
            # =================================================================
            if pliers_used:
                if best_release_pos is not None:
                    # Calculate targets and angles securely
                    with self.data_lock:
                        id_side = self.get_best_side_pliers_release(best_release_pos[2])
                        pliers_theta = self.pliers[id_side * 4].theta if id_side is not None else 0.0
                        
                    target_angle = best_release_pos[2] - pliers_theta
                    target_pos = (best_release_pos[0], best_release_pos[1])

                    # Calculate the safe path to the release zone!
                    with self.data_lock:
                        if self.is_direct_path_clear(start_pos, target_pos):
                            rel_path = [start_pos, target_pos]
                        else:
                            rel_path, _ = self.get_street_grid_path(start_pos, target_pos, x_min, x_max, y_min, y_max)

                    # Drive the grid path
                    if len(rel_path) > 1:
                        for wp in rel_path[1:]:
                            self.get_logger().info(f"Navigating release waypoint: {wp}")
                            self.move_to(Position(wp[0], wp[1], target_angle))
                            self.wait_for_motion()
                            self.stare_and_update()

                    # Configure dropping actions safely
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

                    # Send commands and wait
                    self.send_plier_cmd(id_active_pliers)
                    self.wait_for_plier()

                    # Clear plier states
                    with self.data_lock:
                        for pl_id in id_active_pliers:
                            self.pliers[pl_id].state = -1
                            
                    continue

                else: 
                    self.get_logger().info("Issue: Cannot find release zone. Going to final zone.")
                    with self.data_lock:
                        final_pos = Position(self.final_zone["x"], self.final_zone["y"], 0.0)
                    self.move_to(final_pos)
                    self.wait_for_motion()
                    id_active_pliers_all = {}
                    for pid, plier in self.pliers.items():
                        if plier.state == -1:
                            continue
                        
                        id_active_pliers_all[pid] = ["drop", plier.state]
                    self.send_plier_cmd(id_active_pliers)
                    self.wait_for_plier()

            # =================================================================
            # 3. GO FOR A TAKE
            # =================================================================
            if pick_crate_ind is not None:
                with self.data_lock:
                    target_ids = self.stacks[stack_id] if stack_id is not None else [pick_crate_ind]
                    pliers_config = self.get_best_side_pliers(len(target_ids))

                if not pliers_config:
                    self.get_logger().warn("Target found but no pliers available. Switching to Release.")
                    continue
                
                selected_pliers_ids = pliers_config[0]
    
                with self.data_lock:
                    target_pos = self.get_mean_pose([self.haz_crates[pid] for pid in target_ids])
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

                # --- Calculate Best Camera Aim for the Entry Point ---
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

                # A. Drive the Grid Path (Aiming the camera on the final step!)
                if len(pick_path) > 1:
                    for i, wp in enumerate(pick_path[1:]):
                        if activate_check_stack and i == len(pick_path[1:]) - 1:
                            # Last step: Drift in aiming the camera!
                            self.get_logger().info(f"Aiming camera while arriving at entry point: {wp}")
                            self.move_to(Position(wp[0], wp[1], best_camera_theta))
                        else:
                            # Normal grid steps
                            self.move_to(Position(wp[0], wp[1], final_robot_theta))
                        self.wait_for_motion()
                        self.stare_and_update() # Kept from your old code to update map on the way!

                # B. --- VERIFICATION ---
                # We are parked at the entry point. Make sure it's not a ghost!
                if activate_check_stack:
                    is_ghost = False
                    with self.data_lock:
                        for cid in target_ids:
                            if cid not in self.haz_crates:
                                is_ghost = True
                                break
                                
                    if is_ghost:
                        self.get_logger().warn("Crate missing after approach! Aborting pick...")
                        continue 

                # C. Realignment (Rotate back to align the pliers with the crate)
                # FIX: ONLY realign if the robot actually needs to turn! (Prevents 0-distance hang)
                if not activate_check_stack or abs(self.angular_distance(best_camera_theta, final_robot_theta)) > 0.05:
                    self.move_to(Position(entry_point[0], entry_point[1], final_robot_theta))
                    self.wait_for_motion()

                # D. Plunge into the final position!
                self.move_to(final_pos)
                self.wait_for_motion()

                # E. Activate pliers
                with self.data_lock:
                    robot_x, robot_y, robot_t = final_pos.x, final_pos.y, final_pos.t
                    pliers_in_map = {}
                    for pl_id in selected_pliers_ids:
                        p_local = self.pliers[pl_id]
                        rx, ry = self.rotate_point(p_local.x, p_local.y, robot_t)
                        pliers_in_map[pl_id] = (robot_x + rx, robot_y + ry)

                    # --- THE FIX: Filter out any ghosts that were deleted during the drive ---
                    remaining_targets = [tid for tid in target_ids if tid in self.haz_crates]
                    
                    # Safety check: If ALL crates in the stack turned out to be ghosts, abort!
                    if not remaining_targets:
                        self.get_logger().warn("All targets vanished during approach! Aborting plier activation.")
                        continue 
                    # -------------------------------------------------------------------------

                    dict_sel_pliers = {}
                    for pl_id, (px, py) in pliers_in_map.items():
                        if not remaining_targets: continue
                        
                        best_obj_id = min(
                            remaining_targets, 
                            key=lambda obj_id: (self.haz_crates[obj_id].x - px)**2 + (self.haz_crates[obj_id].y - py)**2
                        )
                        remaining_targets.remove(best_obj_id)
                        dict_sel_pliers[pl_id] = ["pick", best_obj_id]

                # Send commands and wait
                self.send_plier_cmd(dict_sel_pliers)
                self.wait_for_plier()
                continue
            
            # =================================================================
            # 4. EXPLORE / STEAL
            # =================================================================
            else:
                self.get_logger().info("Nothing to do: Exploring / Stealing...")
                pos = steal_poses[id_steal % len(steal_poses)]
                
                # Check boundaries safely
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

    def get_current_zone(self, x, y):
        """
        Checks which square the point (x, y) is inside.
        
        squares: dict like { 'zone_1': {'x': 1.0, 'y': 1.5, 'size': 0.5}, ... }
        Returns the ID of the zone, or None if the point is outside all zones.
        """
        for zone_id, sq in self.zones.items():
            cx = sq.x
            cy = sq.y
            half_size = sq.size / 2.0  # Calculate half-size once per square
            
            # Check if X is within bounds AND Y is within bounds
            if (cx - half_size <= x <= cx + half_size) and \
            (cy - half_size <= y <= cy + half_size):
                return zone_id  # We found the zone! Return it immediately.
                
        return None  # The loop finished without finding a match

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
    
    def get_all_points_in_zone(self, points, zone_x, zone_y, zone_size):
        """
        Finds all points that are inside a specific square zone.
        
        Returns a list of all points found inside the zone.
        """
        half_size = zone_size / 2.0
        min_x = zone_x - half_size
        max_x = zone_x + half_size
        min_y = zone_y - half_size
        max_y = zone_y + half_size
        
        points_inside = []
        
        for p in points:
            if (min_x <= p.x <= max_x) and (min_y <= p.y <= max_y):
                points_inside.append(p)  # Add it to our list and keep checking the rest
                
        return points_inside

    def rotate_point(self, x, y, theta):
        """Applique une rotation 2D simple."""
        cos_t, sin_t = np.cos(theta), np.sin(theta)
        rx = x * cos_t - y * sin_t
        ry = x * sin_t + y * cos_t
        return rx, ry

    def compute_release_rewards(self):
        if not self.any_plier_used():
            return None, None
        max_reward = float('-inf')
        best_zone_id = None
        best_pos = None
        for id, zone in self.zones.items():
            if self.is_any_point_in_zone(id):
                continue
            x = zone.x
            y = zone.y

            distance = 0.26
            av_poses = [[x + distance, y, 3.14],
                        [x - distance, y, 0.0],
                        [x, y + distance, 4.71],
                        [x, y - distance, 1.57],
                    ]

            for pos in av_poses:
                if not (self.boundaries[0] + 0.2 < pos[0] < self.boundaries[1] - 0.2 and self.boundaries[2] + 0.2 < pos[1] < self.boundaries[3] - 0.2): 
                    continue
                reward = self.compute_release_penality(x, y)

                # 5. Mise à jour du champion global
                if reward > max_reward:
                    max_reward = reward
                    best_zone_id = id
                    best_pos = pos
        return best_zone_id, best_pos

    def compute_release_penality(self, x, y):
        coeff_center = 1 # -0.005
        coeff_dst = -1
        coeff_enn = -5 # 0.05
        # coeef_end = -0.0001
        
        val_center = (1.5 - x) ** 2 + (1 - y) ** 2
        val_dst = (self.robot_pos.x - x) ** 2 + (self.robot_pos.y - y) ** 2
        if self.x_enn is not None:
            val_ennemi = (self.x_enn - x) ** 2 + (self.y_enn - y) ** 2
        else:
            val_ennemi = 0
        return coeff_dst * val_dst + coeff_enn * val_ennemi + coeff_center * val_center

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
            return None, None, None, []

        min_total_dist = float('inf')
        best_crate_id = None
        target_stack_id = None
        use_inverted_approach = False
        best_path = []
        
        approach_distance = 0.3
        reviewed_ids = set()
        
        start_pos = (self.robot_pos.x, self.robot_pos.y)
        x_min, x_max, y_min, y_max = self.rect

        for crate_id, crate in self.haz_crates.items():
            if crate_id in reviewed_ids:
                continue

            # 1. Filtres de sécurité
            if crate.state >= 0 or (self.get_current_zone(crate.x, crate.y) is not None and (crate.color in (-1, self.color, 2))): 
                reviewed_ids.add(crate_id)
                continue
            
            # 2. Consolidation des données (Caisse seule ou Pile)
            stack_id = self.get_stack_linked(crate_id)
            if stack_id is not None:
                group_ids = self.stacks[stack_id]
                group_crates = [self.haz_crates[pid] for pid in group_ids]
                mean_x = np.mean([c.x for c in group_crates])
                mean_y = np.mean([c.y for c in group_crates])
                mean_theta = np.arctan2(np.sum([np.sin(c.theta) for c in group_crates]), np.sum([np.cos(c.theta) for c in group_crates]))
                for pid in group_ids: reviewed_ids.add(pid)
            else:
                mean_x, mean_y, mean_theta = crate.x, crate.y, crate.theta
                reviewed_ids.add(crate_id)

            # 3. Calcul des points d'entrée
            entries = [
                (mean_x - approach_distance * np.cos(mean_theta), mean_y - approach_distance * np.sin(mean_theta), False),
                (mean_x + approach_distance * np.cos(mean_theta), mean_y + approach_distance * np.sin(mean_theta), True)
            ]

            for ex, ey, is_inv in entries:
                if not (self.boundaries[0] <= ex <= self.boundaries[1] and self.boundaries[2] <= ey <= self.boundaries[3]):
                    continue
                    
                target_pos = (ex, ey)
                
                # 4. Check Path! Line of sight, then Street Grid
                if self.is_direct_path_clear(start_pos, target_pos, target_crate_id=crate_id):
                    dist = math.hypot(ex - start_pos[0], ey - start_pos[1])
                    path = [start_pos, target_pos]
                else:
                    path, dist = self.get_street_grid_path(start_pos, target_pos, x_min, x_max, y_min, y_max)
                    # Security Check: Dive blocked?
                    if len(path) >= 2 and not self.is_direct_path_clear(path[-2], target_pos, target_crate_id=crate_id):
                        dist = float('inf')

                # 5. Save the absolute best one
                if dist < min_total_dist:
                    min_total_dist = dist
                    best_crate_id = crate_id
                    target_stack_id = stack_id
                    use_inverted_approach = is_inv
                    best_path = path
                            
        return best_crate_id, target_stack_id, use_inverted_approach, best_path

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
        
        # Helper to round coordinates (prevents floating point errors in the graph)
        def pt(x, y): return (round(x, 4), round(y, 4))
        
        start = pt(*start)
        target = pt(*target)
        nodes = set([start, target])
        
        x_lines = [x_min, x_max]
        y_lines = [y_min, y_max]
        
        # 1. Add the 4 main intersections
        for x in x_lines:
            for y in y_lines:
                nodes.add(pt(x, y))
                
        # 2. Project Start and Target onto the lines (ONLY if within board boundaries)
        for p in [start, target]:
            for x in x_lines:
                if self.boundaries[2] <= p[1] <= self.boundaries[3]: nodes.add(pt(x, p[1]))
            for y in y_lines:
                if self.boundaries[0] <= p[0] <= self.boundaries[1]: nodes.add(pt(p[0], y))
                    
        nodes = list(nodes)
        edges = {n: [] for n in nodes}
        
        def add_edge(n1, n2):
            if n1 != n2:
                dist = math.hypot(n1[0]-n2[0], n1[1]-n2[1])
                edges[n1].append((dist, n2))
                edges[n2].append((dist, n1))

        # 3. Connect nodes that share the same horizontal or vertical "Street"
        for i, n1 in enumerate(nodes):
            for n2 in nodes[i+1:]:
                if n1[0] == n2[0] and n1[0] in x_lines: add_edge(n1, n2)
                elif n1[1] == n2[1] and n1[1] in y_lines: add_edge(n1, n2)
                    
        # 4. Connect Start and Target to their respective projections
        for x in x_lines:
            if self.boundaries[2] <= start[1] <= self.boundaries[3]: add_edge(start, pt(x, start[1]))
            if self.boundaries[2] <= target[1] <= self.boundaries[3]: add_edge(target, pt(x, target[1]))
        for y in y_lines:
            if self.boundaries[0] <= start[0] <= self.boundaries[1]: add_edge(start, pt(start[0], y))
            if self.boundaries[0] <= target[0] <= self.boundaries[1]: add_edge(target, pt(target[0], y))

        # 5. Mini-Dijkstra Pathfinding
        queue = [(0, start, [start])]
        visited = set()
        
        while queue:
            cost, current, path = heapq.heappop(queue)
            if current == target:
                # Clean up the path (remove unnecessary middle points on straight lines)
                cleaned = []
                for p in path:
                    if len(cleaned) >= 2:
                        dx1, dy1 = cleaned[-1][0] - cleaned[-2][0], cleaned[-1][1] - cleaned[-2][1]
                        dx2, dy2 = p[0] - cleaned[-1][0], p[1] - cleaned[-1][1]
                        if abs(dx1*dy2 - dx2*dy1) < 1e-4: cleaned.pop() # Remove collinear point
                    cleaned.append(p)
                return cleaned, cost
                
            if current in visited: continue
            visited.add(current)
            
            for d, neighbor in edges[current]:
                if neighbor not in visited:
                    heapq.heappush(queue, (cost + d, neighbor, path + [neighbor]))
                    
        return [], float('inf') # Return infinity if completely trapped

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
        """Checks if the straight line between two points hits any crates."""
        x1, y1 = start_pos
        x2, y2 = target_pos
        dx = x2 - x1
        dy = y2 - y1
        length_sq = dx**2 + dy**2
        
        if length_sq == 0:
            return True
            
        for cid, crate in self.haz_crates.items():
            # Ignore the target crate and crates already picked up
            if cid == target_crate_id or crate.state != -1:
                continue

            # Project crate center onto the line segment to find closest point
            t = max(0.0, min(1.0, ((crate.x - x1) * dx + (crate.y - y1) * dy) / length_sq))
            closest_x = x1 + t * dx
            closest_y = y1 + t * dy
            
            # If the closest point is smaller than our margin, the path is blocked!
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