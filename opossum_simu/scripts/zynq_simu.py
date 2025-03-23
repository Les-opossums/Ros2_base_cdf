#!/usr/bin/env python3

"""Simulate the communication with the Zynq."""

# Libraries import
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException
from ament_index_python.packages import get_package_share_directory
import os
import yaml
from opossum_simu.RobotState import RobotState
import numpy as np
import time

# Msgs import
from std_msgs.msg import String

class ZynqSimulation(Node):
    """Simulate the communication between the zynq and the raspi."""

    def __init__(self: Node) -> None:

        super().__init__("zynq_simu_node")
        self.declare_parameters(
            namespace="",
            parameters=[
                # The 3 follwing are used to make sure the world positions well the robot
                ("real_position_topic", rclpy.Parameter.Type.STRING),
                ("trigger_position_srv", rclpy.Parameter.Type.STRING),
                ("compute_period", rclpy.Parameter.Type.DOUBLE),

                # Others represents the robot conditions 
                ("rcv_comm_topic", rclpy.Parameter.Type.STRING),
                ("send_comm_topic", rclpy.Parameter.Type.STRING),
                ("boundaries", rclpy.Parameter.Type.DOUBLE_ARRAY),
                ("random_moves", rclpy.Parameter.Type.BOOL),
                ("angular_velocity", rclpy.Parameter.Type.DOUBLE),
                ("linear_velocity", rclpy.Parameter.Type.DOUBLE),
            ],
        )
        self._init_parameters()
        self._init_subscribers()
        self._init_publishers()
        self.get_logger().info("ZYNQ simulation node initialized.")

    def _init_parameters(self: Node) -> None:
        """Initialise parameters of the node."""
        is_random = self.get_parameter("random_moves").get_parameter_value().bool_value
        angular_velocity = self.get_parameter("angular_velocity").get_parameter_value().double_value
        linear_velocity = self.get_parameter("linear_velocity").get_parameter_value().double_value
        boundaries = self.get_parameter("boundaries").get_parameter_value().double_array_value
        self.compute_period = self.get_parameter("compute_period").get_parameter_value().double_value
        self.robot_state = RobotState(is_random, linear_velocity, angular_velocity, boundaries)
        msgs_path = os.path.join(
            get_package_share_directory("cdf_msgs"), "resources"
        )
        self.timers_clb = {}
        comm_yaml_file = os.path.join(msgs_path, "com_msgs.yaml") 
        msgs_yaml_file = os.path.join(msgs_path, "format_msgs.yaml")
        with open(comm_yaml_file, "r") as file:
            self.comm_yaml = yaml.safe_load(file)
        with open(msgs_yaml_file, "r") as file:
            self.msgs_yaml = yaml.safe_load(file)

    def _init_subscribers(self: Node) -> None:
        """Initialise the subscribers of the node."""
        self.rcv_comm_topic = (
            self.get_parameter("rcv_comm_topic").get_parameter_value().string_value
        )

        callback_group = ReentrantCallbackGroup()
        self.sub_comm_topic = self.create_subscription(
            String,
            self.rcv_comm_topic,
            self.do_action_callback,
            10,
            callback_group = callback_group,
        )

    def _init_publishers(self: Node) -> None:
        """Initialize publishers of the node."""
        self.send_comm_topic = (
            self.get_parameter("send_comm_topic").get_parameter_value().string_value
        )
        self.pub_comm_topic = self.create_publisher(String, self.send_comm_topic, 10)

    def do_action_callback(self, msg):
        "Do action needed corresponding to messages."
        self.get_logger().info(f"REQ: {msg.data}.")
        parser = msg.data.split()
        name = parser[0]
        args = parser[1:]
        if name not in self.comm_yaml:
            self.get_logger().info(f"The name {name} does not exists in the yaml file com_msgs.")
        order = self.comm_yaml[name]
        msg_type = order["send"]
        if msg_type not in self.msgs_yaml:
            self.get_logger().info(f"The message type {msg_type} does not exists in the yaml file format_msgs.")
        args = {key: args[i] for i, key in enumerate(self.msgs_yaml[msg_type]["struct"].keys())}
        result = self.process_action(name, args)

        if order["receive"] == "null":
            return
        command = name
        msg_type_rcv = order["receive"]
        for key in self.msgs_yaml[msg_type_rcv]["struct"].keys():
            command += " " + str(result[key])
        msg.data = command
        self.pub_comm_topic.publish(msg)
    
    def process_action(self, name, args):
        if name == "GETODOM":
            if int(args["enable"]) == 1 and float(args["freq"]) != 0:
                # self.timers["GETODOM"] = self.create_timer(args["freq"], self.get_odom())
                pass
            else: 
                pass
            result = {
                "x": float(self.robot_state.position[0].item()), 
                "y": float(self.robot_state.position[1].item()), 
                "t": self.robot_state.angle, 
            }
        elif name == "SETODOM":
            self.get_logger().info(f"HERE")
            obj = np.array([[float(args["x"])], [float(args["y"])], [float(args["t"])]])
            real_time = time.time()
            self.get_logger().info(f"{self.robot_state.angle}")
            self.get_logger().info(f"{float(obj[2].item())}")
            while np.linalg.norm(self.robot_state.position[:2] - obj[:2]) > 0.005 or abs(self.robot_state.angle - float(obj[2].item())) > 0.01:
                self.robot_state.move(obj, real_time)
                real_time = time.time()
                time.sleep(self.compute_period)
            result = {"is_done": True}
            self.get_logger().info(f"IM FINALLY END ")
        return result

def main(args=None):
    """Spin main loop."""
    rclpy.init(args=args)
    zynq_comm_simu_node = ZynqSimulation()
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(zynq_comm_simu_node, executor=executor)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        zynq_comm_simu_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()