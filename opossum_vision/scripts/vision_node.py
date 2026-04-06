#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
import threading
import time

# Assurez-vous que l'import fonctionne selon votre structure
try:
    from opossum_msgs.msg import VisionData, VisionDataFrame, CameraLoc
except ImportError:
    class VisionData: pass
    class VisionDataFrame: pass
    class CameraLoc: pass

class SingleVisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        
        # 1. Déclaration des paramètres pour UNE SEULE caméra
        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("camera_id", 1)
        self.declare_parameter("simulation", False)
        
        self.port = self.get_parameter("port").get_parameter_value().string_value
        self.baudrate = self.get_parameter("baudrate").get_parameter_value().integer_value
        self.camera_id = self.get_parameter("camera_id").get_parameter_value().integer_value
        self.simulation = self.get_parameter("simulation").get_parameter_value().bool_value
        
        # 2. Initialisation des Publishers
        self.aruco_pub = self.create_publisher(VisionDataFrame, 'aruco_loc', 10)
        self.camera_log_pub = self.create_publisher(CameraLoc, 'camera_loc', 10)
        
        # 3. Démarrage
        self.is_running = True
        self.serial_card = None
        
        if not self.simulation:
            self.serial_card = self._init_cam()
            if self.serial_card:
                self.read_thread = threading.Thread(target=self._serial_read_worker)
                self.read_thread.start()
        else:
            self.get_logger().info(f"Caméra {self.camera_id} lancée en mode SIMULATION.")

    def _init_cam(self):
        """Initialise la connexion série de manière sécurisée pour ROS 2."""
        while self.is_running:
            self.get_logger().info(f"Tentative de connexion sur {self.port} (Baudrate: {self.baudrate})...")
            try:
                tested_serial = serial.Serial(
                    port=self.port, 
                    baudrate=self.baudrate, 
                    timeout=1.0
                )
                time.sleep(0.2) # Courte pause pour laisser le temps au handshake
                all_data = tested_serial.read_until(b'\n').decode("utf-8", errors="ignore")
                
                self.get_logger().info(f"Caméra {self.camera_id} connectée ! Réponse: {all_data.strip()}")
                return tested_serial
                
            except serial.SerialException as e:
                self.get_logger().warn(f"Échec sur {self.port}. Nouvel essai dans 1s... ({e})")
                time.sleep(1.0)
                
        return None

    def _serial_read_worker(self):
        """Thread dédié à la lecture série bloquante."""
        self.get_logger().info(f"Thread d'écoute démarré sur {self.port}.")
        
        while rclpy.ok() and self.is_running:
            try:
                raw_line = self.serial_card.read_until(b'\n')
                if raw_line:
                    line = raw_line.decode('utf-8', errors="ignore").strip()
                    if line:
                        self._handle_received_line(line)
            except serial.SerialException as e:
                self.get_logger().error(f"Port série déconnecté sur {self.port}: {e}")
                break # Sort de la boucle, le node devra être relancé

    def _handle_received_line(self, data):
        if data and data[0].isalpha():
            self.process_data_rcv(data)

    def process_data_rcv(self, data):
        """Traite les données brutes et publie les messages ROS."""
        if len(data) == 0:
            return
        splitted_data = data.split()
        if len(splitted_data) < 1:
            return
            
        if splitted_data[0] == "ARUCO":  
            parts = data.split(',')
            if len(parts) < 1:
                return

            header_tokens = parts[0].split()
            if len(header_tokens) < 2:
                return

            vision_frame_msg = VisionDataFrame()
            try:
                # Utilise l'ID envoyé par la carte, sinon l'ID du paramètre
                vision_frame_msg.id = int(header_tokens[1]) 
            except ValueError:
                vision_frame_msg.id = self.camera_id

            vision_frame_msg.object = []

            for part in parts[1:]:
                tag_tokens = part.split()
                if tag_tokens:
                    obj = self.create_vision_data(tag_tokens)
                    if obj:
                        vision_frame_msg.object.append(obj)

            self.aruco_pub.publish(vision_frame_msg)

        elif splitted_data[0] == "LOC" and len(splitted_data) == 5: 
            try:
                loc_msg = CameraLoc()
                loc_msg.robot_position.x = float(splitted_data[1]) / 1000.0
                loc_msg.robot_position.y = float(splitted_data[2]) / 1000.0
                loc_msg.robot_position.z = float(splitted_data[3]) 
                loc_msg.camera_id = int(splitted_data[4])
                self.camera_log_pub.publish(loc_msg)
            except Exception as e:
                self.get_logger().warn(f"Erreur parsing LOC: {e}")

        elif splitted_data[0] == "ERROR":
            self.get_logger().error(f"Erreur de la carte: {data}")

    def create_vision_data(self, tokens):
        """Convertit les tokens bruts en un objet VisionData."""
        if len(tokens) < 5: 
            return None

        vd = VisionData()
        vd.name = "" 
        
        try:
            vd.id = int(tokens[0])
            vd.x = float(tokens[1]) / 1000.0
            vd.y = float(tokens[2]) / 1000.0
            vd.z = float(tokens[3]) / 1000.0
            vd.theta = float(tokens[4]) * 3.14159265 / 180.0 + 3.14156 / 2 
            
            if len(tokens) >= 8:
                vd.in_stack = int(float(tokens[5])) 
                vd.stack_id = int(float(tokens[6]))
                vd.in_center = float(tokens[7])
            else:
                vd.in_stack = 0
                vd.stack_id = 0
                vd.in_center = 0.0

        except ValueError as e:
            self.get_logger().warn(f"Erreur de conversion tag: {e}")
            return None
            
        return vd

    def destroy_node(self):
        """Surcharge pour fermer proprement le thread et le port série."""
        self.get_logger().info("Arrêt demandé. Fermeture du matériel...")
        self.is_running = False
        if hasattr(self, 'read_thread') and self.read_thread.is_alive():
            self.read_thread.join(timeout=2.0)
        
        if self.serial_card and self.serial_card.is_open:
            self.serial_card.close()
            self.get_logger().info(f"Port {self.port} fermé proprement.")
            
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SingleVisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()