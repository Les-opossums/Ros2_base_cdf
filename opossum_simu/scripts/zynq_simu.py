#!/usr/bin/env python3
"""Simulate the communication with the Zynq."""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from ament_index_python.packages import get_package_share_directory
import os
import yaml
from rclpy.action import ActionClient
from opossum_msgs.srv import StringReq
from opossum_msgs.action import StringAction
from std_msgs.msg import String


class ZynqSimulation(Node):
    """Simulate the communication between the zynq and the raspi."""

    def __init__(self) -> None:
        super().__init__("zynq_simu_node")
        self.declare_parameters(
            namespace="",
            parameters=[
                ("rcv_comm_topic", rclpy.Parameter.Type.STRING),
                ("send_comm_topic", rclpy.Parameter.Type.STRING),
                ("short_motor_srv", rclpy.Parameter.Type.STRING),
                ("robot_components", rclpy.Parameter.Type.STRING_ARRAY),
            ],
        )
        self._init_parameters()
        self._init_subscribers()
        self._init_publishers()
        self._init_action_clients()
        self._init_service_clients()
        self.get_logger().info("ZYNQ simulation node initialized.")

    def _init_parameters(self):
        """Initialise parameters of the node."""
        self.robot_components = (
            self.get_parameter("robot_components")
            .get_parameter_value()
            .string_array_value
        )
        msgs_path = os.path.join(get_package_share_directory("opossum_msgs"), "resources")
        self.timers_clb = {}
        comm_yaml_file = os.path.join(msgs_path, "com_msgs.yaml")
        msgs_yaml_file = os.path.join(msgs_path, "format_msgs.yaml")
        self.type_names = ["struct", "variable", "array"]
        with open(comm_yaml_file, "r") as file:
            self.comm_yaml = yaml.safe_load(file)
        with open(msgs_yaml_file, "r") as file:
            self.msgs_yaml = yaml.safe_load(file)

    def _init_subscribers(self) -> None:
        """Initialize the subscribers of the node."""
        self.rcv_comm_topic = self.get_parameter("rcv_comm_topic").value
        callback_group = ReentrantCallbackGroup()
        self.sub_comm_topic = self.create_subscription(
            String,
            self.rcv_comm_topic,
            self.do_action_callback,
            10,
            callback_group=callback_group,
        )

    def _init_publishers(self) -> None:
        """Initialize publishers of the node."""
        self.send_comm_topic = self.get_parameter("send_comm_topic").value
        self.pub_comm_topic = self.create_publisher(String, self.send_comm_topic, 10)

    def _init_action_clients(self):
        """Initialize action clients of the node."""
        callback_group = ReentrantCallbackGroup()
        self.robot_action_clients = {
            component: ActionClient(
                self,
                StringAction,
                f"{component}/action_simu",
                callback_group=callback_group,
            )
            for component in self.robot_components
        }

    def _init_service_clients(self):
        """Initialize service clients of the node."""
        callback_group = ReentrantCallbackGroup()
        short_motor_srv = (
            self.get_parameter("short_motor_srv").get_parameter_value().string_value
        )
        self.short_motor_srv = self.create_client(
            StringReq, short_motor_srv, callback_group=callback_group
        )
        # Wait for the service asynchronously (or log warnings until it's available)
        while not self.short_motor_srv.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("No position service available, retrying...")

    def do_action_callback(self, msg: String):
        """Handle an incoming command message and initiate the appropriate action/service call."""
        parser = msg.data.split()
        name = parser[0]
        args_list = parser[1:]
        if name not in self.comm_yaml:
            self.get_logger().info(
                f"The name {name} does not exist in the YAML file com_msgs."
            )
            return
        order = self.comm_yaml[name]
        msg_type = order["send"]
        if msg_type is None:
            self.process_action(name, {})

        elif msg_type not in self.msgs_yaml:
            self.get_logger().info(
                f"The message type {msg_type} does not exist in the YAML file format_msgs."
            )

        else:
            name_type = [
                nt for nt in self.type_names if nt in self.msgs_yaml[msg_type].keys()
            ][0]
            args = {
                key: args_list[i]
                for i, key in enumerate(self.msgs_yaml[msg_type][name_type].keys())
            }
            self.process_action(name, args)

    def process_action(self, name, args):
        """Execute the requested action asynchronously."""
        if (
            name == "GETODOM"
            or name == "SPEED"
            or name == "MAPASSERV"
            or name == "BLOCK"
        ):
            self.send_short_cmd_motor(name, args)
        elif name == "MOVE":
            self.send_long_cmd_motor(name, args)
        elif name == "SETLIDAR":
            pass
        else:
            self.get_logger().warn(f"Action {name} is not implemented.")

    # Actions handler
    def send_long_cmd_motor(self, name, args):
        """Send goal for motors asynchronously."""
        goal = StringAction.Goal()
        goal.data = name
        for arg in args.values():
            goal.data += "," + arg
        client = self.robot_action_clients.get("motors")
        if client is None:
            self.get_logger().error("No action client found for motors!")
            return
        if not client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Motor action server not available!")
            return
        future = client.send_goal_async(goal)
        future.add_done_callback(self.handle_long_cmd_motor_result)

    def handle_long_cmd_motor_result(self, future):
        """Handle the result from the motor action asynchronously."""
        try:
            result_response = future.result()
            # Check if goal was accepted
            if not result_response.accepted:
                self.get_logger().error("Motor action goal was rejected!")
                return
            result_future = result_response.get_result_async()
            result_future.add_done_callback(self.handle_long_motor_action_result)
        except Exception as e:
            self.get_logger().error(f"Motor action send failed: {e}")

    def handle_long_motor_action_result(self, future):
        """Process the final result of the motor action."""
        try:
            res = future.result().result.response
            if res == "":
                return
            res = " ".join(res.split(","))
            result_msg = String()
            result_msg.data = res + "\n"
            self.pub_comm_topic.publish(result_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to get motor action result: {e}")

    # Services handler
    def send_short_cmd_motor(self, name, args):
        """Call a service to get the robot's position asynchronously."""
        req = StringReq.Request()
        req.data = name
        for arg in args.values():
            req.data += "," + arg
        future = self.short_motor_srv.call_async(req)
        future.add_done_callback(self.handle_short_cmd_motor_result)

    def handle_short_cmd_motor_result(self, future):
        """Handle the result from the GETODOM service asynchronously."""
        try:
            res = future.result().response
            res = " ".join(res.split(","))
            result_msg = String()
            result_msg.data = res + "\n"
            self.pub_comm_topic.publish(result_msg)
        except Exception as e:
            self.get_logger().error(f"Motor service call failed: {e}")


def main(args=None):
    """Run the main loop."""
    rclpy.init(args=args)
    node = ZynqSimulation()
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(node, executor=executor)
    except (KeyboardInterrupt) as e:
        node.get_logger().info(f"KeyboardInterrupt, shutting down node: {e}.")
    except Exception as e:
        node.get_logger().error(f"Exception in spin: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
