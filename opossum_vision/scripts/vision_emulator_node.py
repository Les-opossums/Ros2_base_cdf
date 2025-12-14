#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
import random
import time
import math

class ObjectEmulator(Node):
    def __init__(self):
        super().__init__('object_serial_emulator')

        # --- Configuration des paramètres ---
        # Port par défaut (à changer selon votre setup : /dev/ttyUSB0 sur Linux, COM3 sur Windows)
        self.declare_parameter('serial_port', '/dev/ttyV0') 
        self.declare_parameter('baudrate', 115200)

        self.port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud = self.get_parameter('baudrate').get_parameter_value().integer_value

        # --- Connexion Série ---
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=1)
            self.get_logger().info(f"Port série ouvert sur {self.port} à {self.baud} bauds")
        except serial.SerialException as e:
            self.get_logger().error(f"Erreur d'ouverture du port série: {e}")
            self.ser = None

        # --- Timer pour l'envoi des données (10 Hz) ---
        self.timer = self.create_timer(0.1, self.publish_serial_data)

    def publish_serial_data(self):
        if self.ser is None or not self.ser.is_open:
            return

        # 1. Émulation des données (Simulation d'un objet qui tourne)
        t = time.time()
        x = 5.0 + 2.0 * math.cos(t)       # Oscille entre 3 et 7
        y = 5.0 + 2.0 * math.sin(t)       # Oscille entre 3 et 7
        z = 1.5                           # Hauteur fixe
        theta = (t % (2 * math.pi))       # Rotation continue 0 à 2pi

        # 2. Formatage du message
        # Format CSV simple: "x,y,z,theta\n"
        # Exemple: "6.24,5.12,1.50,3.14\n"
        msg_str = f"{x:.2f},{y:.2f},{z:.2f},{theta:.2f}\n"

        # 3. Envoi sur le port série (encodage en bytes nécessaire)
        try:
            self.ser.write(msg_str.encode('utf-8'))
            
            # Log pour debug
            # self.get_logger().info(f"Envoyé: {msg_str.strip()}")
        except Exception as e:
            self.get_logger().error(f"Erreur d'écriture: {e}")

    def destroy_node(self):
        # Fermeture propre du port
        if self.ser is not None and self.ser.is_open:
            self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ObjectEmulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()