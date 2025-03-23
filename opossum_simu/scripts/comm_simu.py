#!/usr/bin/env python3

"""Simulate the communication with the Zynq."""

# Libraries import
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException

# Msgs import
from std_msgs.msg import String

class ZynqCommSimulation(Node):
    """Simulate the communication between the zynq and the raspi."""

    def __init__(self: Node) -> None:

        super().__init__("beacon_detector_node")
        self.declare_parameters(
            namespace="",
            parameters=[
                ("rcv_comm_topic", rclpy.Parameter.Type.STRING),
                ("send_comm_topic", rclpy.Parameter.Type.STRING),
            ],
        )
        self._init_parameters()
        self.get_logger().info("Beacon detector node initialized.")

    def _init_parameters(self: Node) -> None:
        """Initialise parameters of the node."""
        pass

    def _init_subscribers(self: Node) -> None:
        """Initialise the subscribers of the node."""
        self.rcv_comm_topic = (
            self.get_parameter("rcv_comm_topic").get_parameter_value().string_value
        )

        self.sub_comm_topic = self.create_subscription(
            String,
            self.comm_topic,
            self.do_action_callback,
            10,
        )

    def _init_publishers(self: Node) -> None:
        """Initialize publishers of the node."""
        self.send_comm_topic = (
            self.get_parameter("send_comm_topic").get_parameter_value().string_value
        )
        self.pub_comm_topic = self.create_publisher(String, self.send_comm_topic, 10)

    def do_action_callback(self, msg):
        "Do action needed corresponding to messages."
        parser = msg.data.split()
        

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