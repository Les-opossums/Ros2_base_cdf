#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import serial
import threading
import os
import yaml
from ament_index_python.packages import get_package_share_directory
# Assurez-vous que l'import fonctionne selon votre structure
from std_msgs.msg import String
try:
    from opossum_msgs.msg import RobotData, VisionData, CameraLoc
except ImportError:
    class RobotData: pass
    class VisionData: pass
    class CameraLoc: pass

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')

        self.declare_parameters(
            namespace="",
            parameters=[
                ("simulation", rclpy.Parameter.Type.BOOL),
                ("send_comm_topic", rclpy.Parameter.Type.STRING),
                ("rcv_comm_topic", rclpy.Parameter.Type.STRING),
                ("comm_state_topic", rclpy.Parameter.Type.STRING),
                ("cards_name", rclpy.Parameter.Type.STRING_ARRAY),
                ("command_topic", rclpy.Parameter.Type.STRING),
                ("feedback_command_topic", rclpy.Parameter.Type.STRING),
                ("robot_data_topic", rclpy.Parameter.Type.STRING),
                ("goal_position_topic", rclpy.Parameter.Type.STRING),
                ("frequency", rclpy.Parameter.Type.DOUBLE),
            ],
        )
        self._init_parameters()
        self._init_publishers()
        self._init_subscribers()
        self._start_reading_threads()

        self.get_logger().info(f"VisionNode démarré dans le namespace '{self.get_namespace()}'")

    def _init_parameters(self: Node) -> None:
        """Initialise parameters of the node."""
        self.get_logger().info("Initializing parameters...")
        msgs_path = os.path.join(
            get_package_share_directory("opossum_msgs"), "resources"
        )
        comm_yaml_file = os.path.join(msgs_path, "com_msgs.yaml")
        msgs_yaml_file = os.path.join(msgs_path, "format_msgs.yaml")
        with open(comm_yaml_file, "r") as file:
            self.comm_yaml = yaml.safe_load(file)
        with open(msgs_yaml_file, "r") as file:
            self.msgs_yaml = yaml.safe_load(file)
        self.mutex_clb = MutuallyExclusiveCallbackGroup()
        self.default_exec = MultiThreadedExecutor()
        self.simulation = (
            self.get_parameter("simulation").get_parameter_value().bool_value
        )
        self.enable_send = True
        self.type_names = ["struct", "variable", "array"]
        if self.simulation:
            self.buffer_simu_rcv = ""
        self.frequency = (
            self.get_parameter("frequency").get_parameter_value().double_value
        )
        self.data_topic = (
            self.get_parameter("robot_data_topic").get_parameter_value().string_value
        )
        self.command_topic = (
            self.get_parameter("command_topic").get_parameter_value().string_value
        )
        self.simulation = (
            self.get_parameter("simulation").get_parameter_value().bool_value
        )
        self.custom_handled_commands = ("ARUCO", "ERROR")
        if True:
            cards_name = (
                self.get_parameter("cards_name")
                .get_parameter_value()  
                .string_array_value
            )
            self.cards = {}
            self.serial_threads = [] # Liste pour garder une référence des threads
            
            for name in cards_name:
                self.get_logger().info(f"Initializing card: {name}")
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
                # Initialisation bloquante de la carte (handshake)
                self.cards[name]["serial"] = self._init_cam(name)

    def _init_publishers(self):
        '''
        Initialize publishers for detected objects.
        '''
        command_topic = (
            self.get_parameter("command_topic")
            .get_parameter_value()
            .string_value
        )

        self.aruco_pub = self.create_publisher(
            VisionData, 
            'aruco_loc', 
            10
        )

        self.camera_log_pub = self.create_publisher(
            CameraLoc,
            'camera_loc',
            10
        )

        self.pub_command = self.create_publisher(
            String,
            command_topic,
            10
        )

    def _init_subscribers(self):
        '''
        Initialize subscribers for robot data.
        '''
        self.data_sub = self.create_subscription(
            RobotData,
            self.data_topic,
            self.robot_data_callback,
            10
        )

        color_topic = (
            self.get_parameter("color_topic")
            .get_parameter_value()
            .string_value
        )

        self.color_sub = self.create_subscription(
            String,
            color_topic,
            self.color_callback,
            10
        )

    def _init_cam(self, name):
        """Initialize serial connection with handshake."""
        # Note: timeout défini à 1s pour le handshake initial, 
        # il sera potentiellement ajusté dans le thread si besoin, 
        # mais read_until gère son propre blocage.
        while True:
            self.get_logger().info("trying to connect to %s" % name)
            try:
                tested_serial = serial.Serial(
                    port=self.cards[name]["port"], 
                    baudrate=self.cards[name]["baudrate"],
                    timeout=1, # Timeout utile pour le handshake initial
                    write_timeout=0,
                )
                # On attend un peu pour la réponse
                rclpy.spin_once(self, timeout_sec=0.2, executor=self.default_exec)
                
                # Lecture initiale simple pour valider la connexion
                all_data = tested_serial.read_until(b'\n').decode("utf-8")
                
                # if all_data:
                if True: 
                    self.get_logger().info(f"Card {name} connected. Version response: {all_data.strip()}")
                    return tested_serial
                    
            except serial.SerialException as e:
                self.get_logger().error(
                    "Scanned serial port is already opened nor existing !"
                )
                print(e)
            except ValueError as e:
                self.get_logger().info("Unknown card found")
                if 'tested_serial' in locals():
                    tested_serial.close()
            
            self.get_logger().warn("Retrying to connect the '%s' card in 1s" % name)
            rclpy.spin_once(self, timeout_sec=1, executor=self.default_exec)

    def _start_reading_threads(self):
        """Start a dedicated thread for each serial card."""
        for name in self.cards.keys():
            # Création d'un thread par carte pour éviter qu'une carte bloque les autres
            thread = threading.Thread(target=self._serial_read_worker, args=(name,), daemon=True)
            thread.start()
            self.serial_threads.append(thread)
            self.get_logger().info(f"Started reading thread for card: {name}")

    def _serial_read_worker(self, name):
        """Worker function running in a separate thread to read serial data."""
        serial_card = self.cards[name]["serial"]
        
        # Configuration du timeout pour read_until :
        # None = bloquant indéfiniment jusqu'à réception
        # X = bloquant X secondes max
        serial_card.timeout = 1.0 

        self.get_logger().info(f"Thread listening on {name} started.")

        while rclpy.ok():
            try:
                # Lecture bloquante jusqu'au caractère de fin de ligne (\n)
                # C'est beaucoup plus efficace que de poller in_waiting
                raw_line = serial_card.read_until(b'\n')
                
                if raw_line:
                    try:
                        line = raw_line.decode('utf-8').strip()
                        if line: # Si la ligne n'est pas vide
                             # Traitement direct (thread-safe pour les publishers ROS2 en Python)
                             self._handle_received_line(line)
                    except UnicodeDecodeError:
                         self.get_logger().warn(f"Decoding error on card {name}")
                
            except serial.SerialException as e:
                self.get_logger().error(f"Serial exception on {name}: {e}")
                break # On sort de la boucle si le port crash
            except Exception as e:
                self.get_logger().error(f"Unexpected error in thread {name}: {e}")

    def _handle_received_line(self, data):
        """Process a single line of received data."""
        if data and data[0].isalpha():
            self.process_data_rcv(data)

    def process_data_rcv(self, data):
        """Publish the asserv data if necessary."""
        if len(data) == 0:
            return
        splitted_data = data.split()
        if len(splitted_data) < 1:
            return
        if (
            splitted_data[0] == "ARUCO" and len(splitted_data) == 9
        ):  # ARUCO id, x, y, z, theta, in_stack
            try:
                p = VisionData()
                p.id = int(splitted_data[1])
                p.x = float(splitted_data[2])/1000
                p.y = float(splitted_data[3])/1000
                p.z = float(splitted_data[4]) 
                p.theta = float(splitted_data[5])*3.14159/180  # Conversion en radians
                p.in_stack = 1 # int(splitted_data[6])
                p.stack_id = 1 # int(splitted_data[7])
                p.in_center = 0. # float(splitted_data[8])
                self.aruco_pub.publish(p)
            except Exception as e:
                self.get_logger().warn(
                    f"The splitted data was {splitted_data} and got: {e}"
                )

        elif (
            splitted_data[0] == "LOC" and len(splitted_data) == 5
        ): # LOC x, y, z, camera_id
            try:
                loc_msg = CameraLoc()
                loc_msg.robot_position.x = float(splitted_data[1])/1000
                loc_msg.robot_position.y = float(splitted_data[2])/1000
                loc_msg.robot_position.z = float(splitted_data[3]) 
                loc_msg.camera_id = int(splitted_data[4])
                self.camera_log_pub.publish(loc_msg)
            except Exception as e:
                self.get_logger().warn(
                    f"The splitted data was {splitted_data} and got: {e}"
                )

        elif (
            splitted_data[0] == "ROBOTPOS"
        ):
            try:
                # Publish data in command
                command_msg = String()
                cam_x = float(splitted_data[1])
                cam_y = float(splitted_data[2])
                cam_theta = float(splitted_data[3])
                cam_delay = float(splitted_data[4])
                cam_bruit_x = float(splitted_data[5])
                cam_bruit_y = float(splitted_data[6])
                cam_bruit_theta = float(splitted_data[7])
                command_msg.data = f"SETCAMERA1 {cam_x} {cam_y} {cam_theta} {cam_delay} {cam_bruit_x} {cam_bruit_y} {cam_bruit_theta}"
                self.pub_command.publish(command_msg)

            except Exception as e:
                self.get_logger().warn(
                    f"The splitted data was {splitted_data} and got: {e}"
                )

        elif splitted_data[0] == "ERROR":
            self.get_logger().error(f"Error: {data}")

    def robot_data_callback(self, msg):
        # self.get_logger().info(f"Reçu des données sur {self.get_namespace()}")
        pass

    def color_callback(self, msg):
        # Write on serial the color received from the IHM
        try:
            color = msg.data
            if color in ["blue", "yellow"]:
                for name, card in self.cards.items():
                    try:
                        card["serial"].write(("COLOR " +color + "\n").encode('utf-8'))
                        self.get_logger().info(f"Sent color '{color}' to card '{name}'")
                    except Exception as e:
                        self.get_logger().error(f"Failed to send color to card '{name}': {e}")
            else:
                self.get_logger().warn(f"Received invalid color: {color}")
        except Exception as e:
            self.get_logger().error(f"Error in color_callback: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()