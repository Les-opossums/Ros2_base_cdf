#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Receive all the world information and send it to every captors (lidars at now)."""

import rclpy
import os
import yaml
import numpy as np
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from ament_index_python.packages import get_package_share_directory

# Import des messages
from opossum_msgs.srv import StringReq
from geometry_msgs.msg import Point
from std_srvs.srv import Trigger
from std_msgs.msg import Int16
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
        self.get_logger().info("Position sender node initialized.")

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
            ],
        )

        self.robot_names = (
            self.get_parameter("robot_names").get_parameter_value().string_array_value
        )
        self.update_period = (
            self.get_parameter("update_period").get_parameter_value().double_value
        )
        objects_path = os.path.join(
            get_package_share_directory("opossum_bringup"), "config", "objects.yaml"
        )
        self.modified_objects = []

        data = yaml.safe_load(open(objects_path, "r"))
        act_angle = 4.17
        # Get the poition of the pumps on the robot
        actuators_types = data['actuators']
        self.actuators = {name: {} for name in self.robot_names}
        cos_a = np.cos(act_angle)
        sin_a = np.sin(act_angle)
        for k, v in actuators_types.items():
            act = Actuators()
            act.name = v['name']
            act.x = cos_a * v['x'] - sin_a * v['y']
            act.y = sin_a * v['x'] + cos_a * v['y']
            act.size = v['size']
            act.state = 'free'
            for name in self.robot_names:
                self.actuators[name][int(k)] = deepcopy(act)

        # Access first stack item
        element_types = data['stacks']
        self.objects = {}
        nb_objects = 0
        # Iterate over all map objects
        for obj in data['map'].values():
            cos_ = np.cos(obj['t'] * np.pi / 2)
            sin_ = np.sin(obj['t'] * np.pi / 2)
            for element in element_types[obj['type']].values():
                x = obj['x'] + element['x'] * cos_ - element['y'] * sin_ 
                y = obj['y'] + element['x'] * sin_ + element['y'] * cos_
                t = (obj['t'] + element['t']) * np.pi / 2
                elem = Objects()
                elem.id = nb_objects
                elem.state = "free"
                elem.type = element['type']
                elem.x = x
                elem.y = y
                elem.theta = t
                self.objects[elem.id] = elem
                self.modified_objects.append(elem.id)
                nb_objects += 1
        self.current_pos = {name: Point() for name in self.robot_names}
        self.current_state = {name: [0, 1] for name in self.robot_names}

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
                Int16,
                "/" + name + "/actuators_state",
                functools.partial(self._update_state, name=name),
                10,
            )
        self.create_timer(self.update_period, self._publish_general_map)

    def _update_state(self, msg, name):
        result = decode_state(msg.data, len(self.actuators[name]))
        for k in self.actuators[name].keys():
            act = self.actuators[name][k]
            if not result[k] and act.state != 'free':
                self.get_logger().info(f"I released {act.state}")
                if act.state != 'running':
                    self.objects[int(act.state)].state = 'free' # We free the object, considering that it has the pos from last time, so its is ok
                act.state = 'free'
            elif result[k] and act.state == 'free':
                act.state = 'running'

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
        for act in self.actuators[name].values():
            if act.state == 'running':
                x_ = act.x * cos_ - act.y * sin_ + msg.x
                y_ = act.x * sin_ + act.y * cos_ + msg.y
                for obj in self.objects.values():
                    if obj.state == 'free' and obj.type == "can":
                        if (obj.x - x_) ** 2 + (obj.y - y_) ** 2 < act.size ** 2:
                            act.state = str(obj.id)
                            obj.state = f"{name}-{act.name}"
                            obj.x = x_
                            obj.y = y_
                            self.modified_objects.append(obj.id)

                            break
            elif act.state != 'free':
                x_ = act.x * cos_ - act.y * sin_ + msg.x
                y_ = act.x * sin_ + act.y * cos_ + msg.y
                obj_taken = self.objects[int(act.state)]
                obj_taken.x = x_
                obj_taken.y = y_
                obj_taken.theta += msg.z - old_theta
                self.modified_objects.append(obj_taken.id)

    def _publish_general_map(self):
        """Publish the postion to every captor who needs information."""
        objects = []
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
    for i, bit in enumerate(state):
        if bit:
            result |= (1 << i)
    return result

def decode_state(value: int, length: int) -> list[bool]:
    """Decode an integer into a list of booleans of given length."""
    return [(value >> i) & 1 == 1 for i in range(length)]

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