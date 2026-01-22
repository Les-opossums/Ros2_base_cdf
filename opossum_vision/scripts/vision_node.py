#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import serial

# Assurez-vous que l'import fonctionne selon votre structure
try:
    from opossum_msgs.msg import RobotData
except ImportError:
    class RobotData: pass

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')

        self._init_cam('/dev/ttyACM0')
        
        # 1. Chargement des paramètres
        # Le topic est relatif (pas de '/' au début), donc il deviendra /nom_robot/robot_data
        self.declare_parameter("robot_data_topic", "robot_data")
        self.data_topic = self.get_parameter("robot_data_topic").get_parameter_value().string_value

        # 2. Création des Pub/Sub
        # On publie sur un topic relatif à notre namespace (ex: /main_robot/objet_loc)
        self.loc_pub = self.create_publisher(Point, "objet_loc", 10)

        # On écoute le topic configuré
        self.data_sub = self.create_subscription(
            RobotData,
            self.data_topic,
            self.robot_data_callback,
            10
        )

        self.get_logger().info(f"VisionNode démarré dans le namespace '{self.get_namespace()}'")
        self.get_logger().info(f"Écoute sur : {self.get_namespace()}/{self.data_topic}")

    def _init_cam(self, name):
        """Initialize serial connection with handshake."""
        # Note: timeout défini à 1s pour le handshake initial, 
        # il sera potentiellement ajusté dans le thread si besoin, 
        # mais read_until gère son propre blocage.
        while True:
            self.get_logger().info("trying to connect to %s" % name)
            try:
                tested_serial = serial.Serial(
                    #self.cards[name]["port"],
                    name='/dev/ttyACM0', 
                    baudrate=115200,
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
            except ValueError:
                self.get_logger().info("Unknown card found")
                if 'tested_serial' in locals():
                    tested_serial.close()
            
            self.get_logger().warn("Retrying to connect the '%s' card in 1s" % name)
            rclpy.spin_once(self, timeout_sec=1, executor=self.default_exec)

    def robot_data_callback(self, msg):
        self.get_logger().info(f"Reçu des données sur {self.get_namespace()}")
        
        # Exemple de traitement : on renvoie un point fictif
        # p = Point()
        # p.x = 10.0
        # self.loc_pub.publish(p)

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