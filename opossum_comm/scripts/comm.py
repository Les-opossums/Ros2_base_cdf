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
from geometry_msgs.msg import Point


class Communication(Node):
    """Simulate the communication between the zynq and the raspi."""

    def __init__(self: Node) -> None:
        super().__init__("beacon_detector_node")
        self.declare_parameters(
            namespace="",
            parameters=[
                ("simulation", rclpy.Parameter.Type.BOOL),
                ("set_asserv", rclpy.Parameter.Type.BOOL),
                ("asserv_topic", rclpy.Parameter.Type.STRING),
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
        self.get_logger().info(f"Simu: {self.simulation}")
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
        self.type_names = ["struct", "variable", "array"]
        self.set_asserv = (
            self.get_parameter("set_asserv").get_parameter_value().bool_value
        )
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
            send_comm_topic = (
                self.get_parameter("send_comm_topic").get_parameter_value().string_value
            )
            self.pub_comm = self.create_publisher(String, send_comm_topic, 10)

        if self.set_asserv:
            asserv_topic = (
                self.get_parameter("asserv_topic").get_parameter_value().string_value
            )
            self.pub_asserv_pos = self.create_publisher(
                Point, asserv_topic + "/pos", 10
            )
            self.pub_asserv_vel = self.create_publisher(
                Point, asserv_topic + "/vel", 10
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
                for line in all_data.split("\n"):
                    self.get_logger().info(f"line: {line}")
                    # splited = line.split(",")
                    if True:
                        self.get_logger().info("GOT IT")

                        # if (
                        #     splited[0].lower() == "version"
                        #     and splited[1][1:].lower() == name
                        # ):
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
                    self.get_logger().info(f"received: {received_data}")
                    try:
                        data_sequences = received_data.split("\n")
                    except Exception:
                        self.get_logger().info("Error While Decoding Data")
                        data_sequences = []
                    for data in data_sequences:
                        self.pub_feedback_command.publish(String(data=data))
                        if self.set_asserv:
                            self.check_and_publish_asserv(data)

        except Exception as e:
            self.get_logger().error("Unhandled Exception : %s" % e)

    def read_card_simu(self, msg):
        """Look at the result of the command send in simulation."""
        data_seq = msg.data.split("\n")
        for data in data_seq:
            self.pub_feedback_command.publish(String(data=data))
            if self.set_asserv:
                self.check_and_publish_asserv(data)

    def send_card_simu(self, msg):
        """Send to the card the command in simulation."""
        self.pub_comm.publish(msg)

    def check_and_publish_asserv(self, data):
        """Publish the asserv data if necessary."""
        if len(data) == 0:
            return
        splitted_data = data.split()
        if len(splitted_data) < 1:
            return
        self.get_logger().info(f"splitted_data: {splitted_data}")
        if splitted_data[0] == "MAPASSERV":
            pos = Point()
            pos.x = float(splitted_data[1])
            pos.y = float(splitted_data[2])
            pos.z = float(splitted_data[3])
            self.pub_asserv_pos.publish(pos)
            vel = Point()
            vel.x = float(splitted_data[4])
            vel.y = float(splitted_data[5])
            vel.z = float(splitted_data[6])
            self.pub_asserv_vel.publish(vel)


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
