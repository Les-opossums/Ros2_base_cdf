#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
# Assurez-vous que l'import fonctionne selon votre structure
try:
    from opossum_msgs.msg import RobotData
except ImportError:
    class RobotData: pass

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        
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