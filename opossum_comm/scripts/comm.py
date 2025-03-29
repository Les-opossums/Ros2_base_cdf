#!/usr/bin/env python3

"""Simulate the communication with the Zynq."""

# Libraries import
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.executors import ExternalShutdownException
from ament_index_python.packages import get_package_share_directory
import os
import serial
import yaml
import functools

# Msgs import
from std_msgs.msg import String


class Communication(Node):
    """Simulate the communication between the zynq and the raspi."""

    def __init__(self: Node) -> None:
        super().__init__("beacon_detector_node")
        self.declare_parameters(
            namespace="",
            parameters=[
                ("simulation", rclpy.Parameter.Type.BOOL),
                ("send_comm_topic", rclpy.Parameter.Type.STRING),
                ("rcv_comm_topic", rclpy.Parameter.Type.STRING),
                ("cards_name", rclpy.Parameter.Type.STRING_ARRAY),
                ("command_topic", rclpy.Parameter.Type.STRING),
                ("feedback_command_topic", rclpy.Parameter.Type.STRING),
                ("frequency", rclpy.Parameter.Type.DOUBLE),
            ],
        )
        self._init_parameters()
        self._init_publishers()
        self._init_subscribers()
        self.get_logger().info("Communication node initialized.")

    def _init_parameters(self: Node) -> None:
        """Initialise parameters of the node."""
        msgs_path = os.path.join(get_package_share_directory("cdf_msgs"), "resources")
        comm_yaml_file = os.path.join(msgs_path, "com_msgs.yaml")
        msgs_yaml_file = os.path.join(msgs_path, "format_msgs.yaml")
        with open(comm_yaml_file, "r") as file:
            self.comm_yaml = yaml.safe_load(file)
        with open(msgs_yaml_file, "r") as file:
            self.msgs_yaml = yaml.safe_load(file)
        self.default_exec = MultiThreadedExecutor()
        self.simulation = (
            self.get_parameter("simulation").get_parameter_value().bool_value
        )
        self.get_logger().info(f"Simu: {self.simulation}")
        if not self.simulation:
            cards_name = (
                self.get_parameter("cards_name")
                .get_parameter_value()
                .string_array_value
            )
            self.frequency = (
                self.get_parameter("frequency").get_parameter_value().double_value
            )
            self.cards = {}
            for name in cards_name:
                self.get_logger().info(f"cards.{name}.port")
                self.declare_parameter(
                    f"cards.{name}.port", rclpy.Parameter.Type.STRING
                )
                port = (
                    self.get_parameter(f"cards.{name}.port")
                    .get_parameter_value()
                    .string_value
                )

                self.declare_parameter(
                    f"cards.{name}.baudrate", rclpy.Parameter.Type.INTEGER
                )
                baudrate = (
                    self.get_parameter(f"cards.{name}.baudrate")
                    .get_parameter_value()
                    .integer_value
                )
                self.cards[name] = {"port": port, "baudrate": baudrate}
                self.cards[name]["serial"] = self._init_card(name)

    def _init_subscribers(self: Node) -> None:
        """Initialise the subscribers of the node."""
        command_topic = (
            self.get_parameter("command_topic").get_parameter_value().string_value
        )
        if self.simulation:
            self.rcv_comm_topic = (
                self.get_parameter("rcv_comm_topic").get_parameter_value().string_value
            )
            self.sub_comm_topic = self.create_subscription(
                String,
                self.rcv_comm_topic,
                self.read_card_simu,
                10,
            )
            self.sub_command = self.create_subscription(
                String, command_topic, self.send_card_simu, 10
            )
        else:
            self.read_timer = self.create_timer(1 / self.frequency, self.read_card)
            # Currently, only one card, so we have name = first card name
            name = list(self.cards.keys())[0]
            self.sub_command = self.create_subscription(
                String, command_topic, functools.partial(self.send_card, name=name), 10
            )

    def _init_publishers(self: Node) -> None:
        """Initialize publishers of the node."""
        if self.simulation:
            self.send_comm_topic = (
                self.get_parameter("send_comm_topic").get_parameter_value().string_value
            )
            self.pub_comm_topic = self.create_publisher(
                String, self.send_comm_topic, 10
            )

        feedback_command_topic = (
            self.get_parameter("feedback_command_topic")
            .get_parameter_value()
            .string_value
        )
        self.pub_feedback_command = self.create_publisher(
            String, feedback_command_topic, 10
        )

    def _init_card(self, name):
        while True:
            self.get_logger().info("trying to connect to %s" % name)
            try:
                tested_serial = serial.Serial(
                    self.cards[name]["port"],
                    self.cards[name]["baudrate"],
                    timeout=0,
                    write_timeout=0,
                )
                tested_serial.write("VERSION\n".encode("utf-8"))
                rclpy.spin_once(self, timeout_sec=0.2, executor=self.default_exec)
                all_data = tested_serial.read(tested_serial.in_waiting).decode("utf-8")
                self.get_logger().info(f"All data: {all_data}")
                self.get_logger().info(f"Name: {name}")
                for line in all_data.split("\n"):
                    splited = line.split(",")
                    if (
                        splited[0].lower() == "version"
                        and splited[1][1:].lower() == name
                    ):
                        return tested_serial
                    else:
                        raise ValueError("unknown card type")
            except serial.SerialException as e:
                self.get_logger().error(
                    "Scanned serial port is already opened nor existing !"
                )
                print(e)
            except ValueError:
                self.get_logger().info("Unknown card found")
                tested_serial.close()
            self.get_logger().warn("Retrying to connect the '%s' card in 1s" % name)
            rclpy.spin_once(self, timeout_sec=1, executor=self.default_exec)

    def send_card(self, msg, name):
        """Send the received message frome ActionSequencer to real card."""
        self.cards[name]["serial"].write((msg.data + "\n").encode("utf-8"))

    def read_card(self):
        """Read the messages that could have been sent by the Zynq."""
        try:
            for name in self.cards.keys():
                serial_card = self.cards[name]["serial"]
                char_available = serial_card.in_waiting
                if char_available != 0:
                    received_data = str(serial_card.read(char_available).decode("UTF8"))
                    self.get_logger().info(f"Received_data: {received_data}")
                    try:
                        data_sequences = received_data.split("\n")
                    except Exception:
                        self.get_logger().info("Error While Decoding Data")
                        data_sequences = []
                    self.get_logger().info(f"data_sequences: {data_sequences}")
                    for data in data_sequences:
                        self.pub_feedback_command.publish(String(data=data))
        except Exception as e:
            self.get_logger().error("Unhandled Exception : %s" % e)

    def read_card_simu(self, msg):
        """Look at the result of the command send in simulation."""
        parser = msg.data.split()
        name = parser[0]
        args = parser[1:]
        if name not in self.comm_yaml:
            self.get_logger().info(
                f"The name {name} does not exists in the yaml file com_msgs."
            )
        msg_type = self.comm_yaml[name]["receive"]
        if msg_type not in self.msgs_yaml:
            self.get_logger().info(
                f"The message type {msg_type} does not exists in the yaml file format_msgs."
            )
        result = {
            key: args[i]
            for i, key in enumerate(list(self.msgs_yaml[msg_type]["struct"].keys()))
        }
        self.get_logger().info("Received:")
        for key in result.keys():
            self.get_logger().info(f"{key}: {result[key]}.")

    def send_card_simu(self, msg):
        """Send to the card the command in simulation."""
        self.pub_comm_topic.publish(msg)


def main(args=None):
    """Spin main loop."""
    rclpy.init(args=args)
    zynq_comm_simu_node = Communication()
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
