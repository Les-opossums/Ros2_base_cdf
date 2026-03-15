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
try:
    from opossum_msgs.msg import RobotData, VisionData, VisionDataFrame, CameraLoc
except ImportError:
    class RobotData: pass
    class VisionData: pass
    class VisionDataFrame: pass
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
        self.aruco_pub = self.create_publisher(
            VisionDataFrame, 
            'aruco_loc', 
            10
        )

        self.camera_log_pub = self.create_publisher(
            CameraLoc,
            'camera_loc',
            10
        )

    def _init_subscribers(self):
        '''
        Initialize subscribers for robot data.
        '''
        pass

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
            splitted_data[0] == "ARUCO"
        ):  
            # Format actuel : ARUCO <CAM_ID> , <INFO_TAG_1>, <INFO_TAG_2>
            parts = data.split(',')
            
            if len(parts) < 1:
                return

            # 1. Extraction de l'ID caméra depuis parts[0] ("ARUCO <CAM_ID>")
            header_tokens = parts[0].split()
            if len(header_tokens) < 2:
                return

            vision_frame_msg = VisionDataFrame()

            try:
                vision_frame_msg.id = int(header_tokens[1]) # cam_id
            except ValueError:
                self.get_logger().error("Erreur de conversion de l'ID caméra.")
                return

            vision_frame_msg.object = []

            # 2. Traitement direct et unifié de tous les tags (à partir de parts[1])
            for part in parts[1:]:
                tag_tokens = part.split()
                if tag_tokens:
                    obj = self.create_vision_data(tag_tokens)
                    if obj:
                        vision_frame_msg.object.append(obj)

            # 3. Publication
            self.aruco_pub.publish(vision_frame_msg)

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

        elif splitted_data[0] == "ERROR":
            self.get_logger().error(f"Error: {data}")

    def create_vision_data(self, tokens):
        """
        Convertit la liste de valeurs brutes en un objet VisionData.
        ATTENTION: Les index (tokens[1], tokens[2]...) doivent correspondre 
        à l'ordre réel de ta trame série.
        """
        if len(tokens) < 5: # Vérification de sécurité minimale (ID, X, Y, Z, Theta)
            return None

        vd = VisionData()
        vd.name = "" # Non utilisé, laissé vide
        
        try:
            vd.id = int(tokens[0])
            vd.x = float(tokens[1])/1000
            vd.y = float(tokens[2])/1000
            vd.z = float(tokens[3])/1000
            vd.theta = float(tokens[4]) * 3.14159265 / 180.0 + 3.14156 / 2 # Conversion en radians
            
            # Pour les attributs suivants, j'assigne des index hypothétiques.
            # Il faudra ajuster ces index [5], [6], [7] selon la position 
            # exacte de "in_stack", "stack_id", et "in_center" dans ta trame.
            if len(tokens) >= 8:
                vd.in_stack = int(float(tokens[5])) # int(float()) au cas où la valeur arriverait comme "1.0"
                vd.stack_id = int(float(tokens[6]))
                vd.in_center = float(tokens[7])
            else:
                # Valeurs par défaut si elles ne sont pas dans la trame
                vd.in_stack = 0
                vd.stack_id = 0
                vd.in_center = 0.0

        except ValueError as e:
            self.get_logger().warn(f"Données ignorées, erreur de conversion: {e}")
            return None
            
        return vd

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