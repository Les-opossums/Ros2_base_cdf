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

# Msgs import
from std_msgs.msg import String

class ZynqCommSimulation(Node):
    """Simulate the communication between the zynq and the raspi."""

    def __init__(self: Node) -> None:

        super().__init__("beacon_detector_node")
        self.declare_parameters(
            namespace="",
            parameters=[
                ("send_comm_topic", rclpy.Parameter.Type.STRING),
                ("rcv_comm_topic", rclpy.Parameter.Type.STRING),
            ],
        )
        self._init_parameters()
        self._init_subscribers()
        self._init_publishers()
        self.get_logger().info("Beacon detector node initialized.")

    def _init_parameters(self: Node) -> None:
        """Initialise parameters of the node."""
        msgs_path = os.path.join(
            get_package_share_directory("cdf_msgs"), "resources"
        )
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

        self.sub_comm_topic = self.create_subscription(
            String,
            self.comm_topic,
            self.result_callback,
            10,
        )

    def _init_publishers(self: Node) -> None:
        """Initialize publishers of the node."""
        self.send_comm_topic = (
            self.get_parameter("send_comm_topic").get_parameter_value().string_value
        )
        self.pub_comm_topic = self.create_publisher(String, self.send_comm_topic, 10)

    def result_callback(self, msg):
        "Look at the result of the command send."
        parser = msg.data.split()
        name = parser[0]
        args = parser[1:]
        if name not in self.comm_yaml:
            self.get_logger().info(f"The name {name} does not exists in the yaml file com_msgs.")
        msg_type = self.comm_yaml[name]["receive"]
        if msg_type not in self.msgs_yaml:
            self.get_logger().info(f"The message type {msg_type} does not exists in the yaml file format_msgs.")
        result = {key: args[i] for i, key in enumerate(self.msgs_yaml)}
        for key in result.keys():
            self.get_logger().info(f"{key}: {result[key]}.")

def main(args=None):
    """Spin main loop."""
    rclpy.init(args=args)
    zynq_comm_simu_node = ZynqCommSimulation()
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