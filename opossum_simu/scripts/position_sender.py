#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Receive all the world information and send it to every captors (lidars at now)."""

import rclpy
import random
import os
import yaml
import numpy as np
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from ament_index_python.packages import get_package_share_directory
from scipy.optimize import linear_sum_assignment
import math

# Import des messages
from opossum_msgs.srv import StringReq
from geometry_msgs.msg import Point
from std_srvs.srv import Trigger
from std_msgs.msg import Int64
from opossum_msgs.msg import PositionMap, RobotData, Objects, GlobalView, Actuators
import functools
from copy import deepcopy

class PositionSender(Node):
    """Receive all the world information and send it to every captors (lidars at now)."""

    def __init__(self):
        super().__init__("position_sender_node")
        self._init_parameters()
        self._init_publishers()
        self._init_subscribers()
        self._init_clients()
        self._init_servers()

    def _init_parameters(self) -> None:
        """Initialize parameters of the node."""
        self.declare_parameters(
            namespace="",
            parameters=[
                ("robot_names", rclpy.Parameter.Type.STRING_ARRAY),
                ("real_position_topic", rclpy.Parameter.Type.STRING),
                ("short_motor_srv", rclpy.Parameter.Type.STRING),
                ("update_period", rclpy.Parameter.Type.DOUBLE),
                ("update_position_topic", rclpy.Parameter.Type.STRING),
                ("board_config", "small_objects"),
                ("year", 2026),
            ],
        )

        self.robot_names = (
            self.get_parameter("robot_names").get_parameter_value().string_array_value
        )
        self.update_period = (
            self.get_parameter("update_period").get_parameter_value().double_value
        )
        self.year = self.get_parameter("year").get_parameter_value().integer_value
        self.board_config = self.get_parameter("board_config").get_parameter_value().string_value
        objects_path = os.path.join(
            get_package_share_directory("opossum_bringup"), "config", str(self.year), f"{self.board_config}.yaml"
        )
        self.modified_objects = []

        

        data = yaml.safe_load(open(objects_path, "r"))
        # act_angle = 4.17
        act_angle = 0.0
        # Get the poition of the pumps on the robot
        actuators_types = data['actuators']
        self.actuators_actions = data['actuators_type']
        # self.actuators_actions = {name: actions for name, actions in actuators_actions_type.items()}
        self.actuators = {name: {} for name in self.robot_names}
        cos_a = np.cos(act_angle)
        sin_a = np.sin(act_angle)
        nb_objects = 0
        for k, v in actuators_types.items():
            act = Actuators()
            act.name = v['name']
            act.type = v['type']
            act.x = cos_a * v['x'] - sin_a * v['y']
            act.y = sin_a * v['x'] + cos_a * v['y']
            act.size = v['size']
            act.theta = v['t'] + act_angle
            act.state = 'free'
            act.state_int = 0
            for name in self.robot_names:
                act.id = nb_objects
                self.actuators[name][int(k)] = deepcopy(act)
                nb_objects += 1

        # Access first stack item
        element_types = data['stacks']
        self.objects = {}
        # Iterate over all map objects
        for obj in data['map'].values():
            cos_ = np.cos(obj['t'])
            sin_ = np.sin(obj['t'])
            if obj['type'] == "full_haz_stack_crates" and obj['shape'] == "clean":
                order = ["yellow", "yellow", "blue", "blue"]
                random.shuffle(order)
            elif obj['type'] == "half_haz_stack_crates" and obj['shape'] == "clean":
                order = ["yellow", "blue"]
                random.shuffle(order)
            else:
                order = ["rot", "rot", "rot", "rot"]
            for ind, element in element_types[obj['type']].items():
                
                x = obj['x'] + element['x'] * cos_ - element['y'] * sin_
                y = obj['y'] + element['x'] * sin_ + element['y'] * cos_
                t = obj['t'] + element['t']
                elem = Objects()
                elem.id = nb_objects
                elem.state = "free"
                if obj['type'] in ("full_haz_stack_crates", "half_haz_stack_crates"):
                    elem.state += "*" + order[ind]
                elem.type = element['type']
                elem.x = x
                elem.y = y
                elem.theta = t
                self.objects[elem.id] = elem
                self.modified_objects.append(elem.id)
                nb_objects += 1
        self.current_pos = {name: Point() for name in self.robot_names}
        self.current_state = {name: [0, 1] for name in self.robot_names}
        self.inv_colors = {"blue": "yellow", "yellow": "blue", "rot": "rot"}

    def _init_publishers(self) -> None:
        """Initialize publishers of the node."""
        self.update_position_topic = (
            self.get_parameter("update_position_topic")
            .get_parameter_value()
            .string_value
        )
        self.pub_update_position = {
            name: self.create_publisher(
                PositionMap, name + "/" + self.update_position_topic, 10
            )
            for name in self.robot_names
        }
        self.pub_global_view = self.create_publisher(
            GlobalView, "global_view", 10
        )

    def _init_subscribers(self):
        """Initialize subscribers of the node."""
        self.real_position_topic = (
            self.get_parameter("real_position_topic").get_parameter_value().string_value
        )

        for name in self.robot_names:
            self.create_subscription(
                Point,
                "/" + name + "/" + self.real_position_topic,
                functools.partial(self._update_pos, name=name),
                10,
            )
            self.create_subscription(
                Int64,
                "/" + name + "/actuators_state",
                functools.partial(self._update_state, name=name),
                10,
            )
        self.create_timer(self.update_period, self._publish_general_map)

    def _update_state(self, msg, name):
        result = decode_state(msg.data, len(self.actuators[name]))
        for k in self.actuators[name].keys():
            act = self.actuators[name][k]
            act.state_int = result[k]
            # 0: free, 1: picking, 2: dropping, 3: reverting, 4: keep up
            if result[k] in (0, 2, 3) and act.state != 'free':
                if act.state not in ('running', 'keeping') :
                    # The state of an object is "free_color" or "free_rot", we want to keep the color information when the object is released
                    if result[k] == 2: # dropping, so we need to update the color of the object to the one of the robot
                        self.objects[int(act.state)].state = 'free' + "*" + self.objects[int(act.state)].state.split("*")[-1]
                    elif result[k] == 3: # picking, so we need to update the color of the object to the inverse of the one of the robot
                        self.objects[int(act.state)].state = 'free' + "*" + self.inv_colors[self.objects[int(act.state)].state.split("*")[-1]]
                    self.modified_objects.append(int(act.state))
                act.state = 'free'
            elif result[k] == 1 and act.state == 'free':
                act.state = 'running'
            elif result[k] == 4 and act.state == 'running':
                act.state = 'keeping'

    def _init_servers(self):
        self.srv = self.create_service(Trigger, 'send_global_data', self._reset_global_objects)

    def _init_clients(self):
        """Initialize clients of the node."""
        components = ["motors"]
        for name in self.robot_names:
            for component in components:
                cli_motor_srv = self.create_client(
                    StringReq, name + "/" + component + "/service_simu"
                )
                while not cli_motor_srv.wait_for_service(timeout_sec=1.0):
                    self.get_logger().warn(f"No data to update currently for the {component} of {name}...")
                req = StringReq.Request()
                if component == "motors":
                    req.data = "GETODOM,1,30.0"
                    future = cli_motor_srv.call_async(req)
                    rclpy.spin_until_future_complete(self, future)
                    res = future.result().response.split(",")
                    self.current_pos[name] = Point(
                        x=float(res[1]), y=float(res[2]), z=float(res[3])
                    )

    def _update_pos(self, msg, name):
        """Update the position of the robot corresponding to the name."""
        old_theta = self.current_pos[name].z
        self.current_pos[name] = msg
        cos_ = np.cos(msg.z)
        sin_ = np.sin(msg.z)

        running_actuators = []

        for act in self.actuators[name].values():
            # Calculate global position of this actuator
            x_ = act.x * cos_ - act.y * sin_ + msg.x
            y_ = act.x * sin_ + act.y * cos_ + msg.y
            theta_ = act.theta + msg.z

            if act.state not in ('free', 'keeping', 'running'):
                # 1. The actuator is ALREADY holding an object. Drag it along!
                obj_taken = self.objects[int(act.state)]
                obj_taken.x = x_
                obj_taken.y = y_
                obj_taken.theta += msg.z - old_theta
                self.modified_objects.append(obj_taken.id)
                
            elif act.state == 'running':
                # 2. The actuator is actively trying to pick something up.
                # Save it in a list so we can run the Hungarian algorithm on all of them at once.
                running_actuators.append((act, x_, y_, theta_))

        # Run the optimal matching algorithm if any actuators are trying to pick
        if running_actuators:
            self._process_hungarian_picking(name, running_actuators)
    
    def _process_hungarian_picking(self, name, running_actuators):
        """Use the Hungarian Algorithm to optimally assign available crates to active actuators."""
        
        # 1. Find all free objects
        available_objects = [
            obj for obj in self.objects.values() 
            if obj.state.split("*")[0] == 'free'
        ]

        if not available_objects:
            return

        num_acts = len(running_actuators)
        num_objs = len(available_objects)
        cost_matrix = np.zeros((num_acts, num_objs))

        # 2. Build the Cost Matrix (Combining Distance + Angle)
        for i, (act, ax, ay, atheta) in enumerate(running_actuators):
            for j, obj in enumerate(available_objects):
                
                # If the actuator isn't allowed to take this type, set an impossibly high cost
                if obj.type not in self.actuators_actions[act.type]["take"]:
                    cost_matrix[i, j] = 1000.0
                    continue

                dist = math.hypot(obj.x - ax, obj.y - ay)

                # Calculate symmetric angular difference
                raw_diff = abs((obj.theta - atheta) % np.pi)
                angle_diff = min(raw_diff, np.pi - raw_diff)

                # The cost is primarily distance, with a slight penalty for bad angles
                # This ensures the closest crate is prioritized, but ties are broken by best angle!
                cost_matrix[i, j] = dist + (angle_diff * 0.05)

        # 3. Compute optimal pairs
        act_indices, obj_indices = linear_sum_assignment(cost_matrix)

        # You can safely increase this tolerance now without fear of them stealing crates!
        tol_angle = 0.6  # (Previously 0.4)

        # 4. Apply the assignments
        for act_idx, obj_idx in zip(act_indices, obj_indices):
            act, ax, ay, atheta = running_actuators[act_idx]
            obj = available_objects[obj_idx]

            # Double check type just in case
            if obj.type not in self.actuators_actions[act.type]["take"]:
                continue

            dist = math.hypot(obj.x - ax, obj.y - ay)
            raw_diff = abs((obj.theta - atheta) % np.pi)
            angle_diff = min(raw_diff, np.pi - raw_diff)

            # If the OPTIMAL match is within our generous tolerances, grab it!
            if dist < act.size and angle_diff < tol_angle:
                act.state = str(obj.id)
                obj.state = f"{name}-{act.name}*" + obj.state.split("*")[-1]
                self.modified_objects.append(obj.id) 

    def _publish_general_map(self):
        """Publish the postion to every captor who needs information."""
        for name in self.robot_names:
            if self.current_pos[name] is not None:
                msg = PositionMap()
                msg.robot = self.current_pos[name]
                for n in [na for na in self.robot_names if na != name]:
                    msg.ennemis.append(self.current_pos[n])
                self.pub_update_position[name].publish(msg)
        msg_global = GlobalView()
        for name in self.robot_names:
            robot = RobotData()
            robot.name = name
            robot.x = self.current_pos[name].x
            robot.y = self.current_pos[name].y
            robot.theta = self.current_pos[name].z
            msg_global.robots.append(robot)
        while len(self.modified_objects) > 0:
            msg_global.objects.append(self.objects[self.modified_objects.pop()])
        mapping_st = {0: "free", 1: "pick", 2: "drop", 3: "revdrop", 4: "keeping"}
        for name in self.robot_names:
            cpos = self.current_pos[name]
            cos_ = np.cos(cpos.z)
            sin_ = np.sin(cpos.z)
            for val in self.actuators[name].values():
                x_ = val.x * cos_ - val.y * sin_ + cpos.x
                y_ = val.x * sin_ + val.y * cos_ + cpos.y
                theta_ = val.theta + cpos.z
                obj = Objects()
                obj.id = val.id
                obj.state = "a*" + mapping_st[val.state_int]
                obj.x = x_
                obj.y = y_
                obj.theta = theta_
                obj.type = val.type
                msg_global.objects.append(obj)
        self.pub_global_view.publish(msg_global)

    def _reset_global_objects(self, request, response):
        for obj in self.objects.keys():
            self.modified_objects.append(obj)
        response.success = True
        return response
    
def _is_bit_set(value: int, index: int) -> bool:
    """Return True if the bit at `index` is set in `value`."""
    return (value >> index) & 1 == 1

def encode_state(state) -> int:
    """Encode a list of booleans into an integer."""
    result = 0
    for i, value in enumerate(state):
        result |= (int(value) << (i * 3))
    return result

def decode_state(value: int, length: int) -> list[int]:
    """Decode an integer into a list of booleans of given length."""
    # Now, i will have not only true of false, but 0: free, 1: picking, 2: dropping, 3: revert_dropping, so need to review the archi, now need 2 bits for each.
    return [(value >> (i * 3)) & 0b111 for i in range(length)]

def main(args=None):
    """Run the main loop."""
    rclpy.init(args=args)
    position_sender_node = PositionSender()
    try:
        rclpy.spin(position_sender_node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        position_sender_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()