import rclpy
from rclpy.node import Node

from cdf_msgs.msg import Obstacles
from sensor_msgs.msg import LaserScan

import obj_detector.trigo as trigo 

import numpy as np

class wall_localisation(Node):
    def __init__(self):
        super().__init__('wall_localisation')
        self.subscription = self.create_subscription(
            Obstacles,
            'bot_obstacle',
            self.wall_localisation_callback,
            1)
        self.subscription_laser = self.create_subscription(
            LaserScan,
            'bottom_lidar/scan',
            self.laser_callback,
            1)
        
        self.subscription
        self.subscription_laser

    def wall_localisation_callback(self, msg : Obstacles):
        liste_segment = []
        for segment in msg.segments:
            segment_temp = [segment.index_first_point,segment.index_last_point]
            if trigo.distance(segment_temp[1],segment_temp[0],self.message_Lidar) > 0.2:
                liste_segment.append([segment_temp[0],segment_temp[1]])
        #self.get_logger().info(f'segments : {liste_segment}')

        rref_l = []
        theta_ex_l = []
        theta_bis_l = []
        A = []
        B = []

        for k in list(range(len(liste_segment))) :
            A_temp,B_temp = trigo.calc_pente(liste_segment[k],self.message_Lidar)
            theta_bis_temp = trigo.calc_theta_bis(liste_segment[k],self.message_Lidar.angle_increment*np.abs(liste_segment[k][0]-liste_segment[k][1]),self.message_Lidar)
            rref_temp = trigo.calc_rref(liste_segment[k][0],theta_bis_temp,self.message_Lidar)
            theta_ex_temp = trigo.cacl_theta_ex(liste_segment[k],theta_bis_temp,self.message_Lidar)
            theta_ex_l.append(theta_ex_temp)
            theta_bis_l.append(theta_bis_temp)
            rref_l.append(rref_temp)
            A.append(A_temp)
            B.append(B_temp)
        #self.get_logger().info(f'A : {A}')
        #self.get_logger().info(f'B : {B}')
        #self.get_logger().info(f'rref : {rref_l}')
        #self.get_logger().info(f'theta_ex : {theta_ex_l}')
        #self.get_logger().info(f'theta_bis : {theta_bis_l}')

    def laser_callback(self, msg : LaserScan):
        self.message_Lidar = msg

def main(args=None):
    rclpy.init(args=None)
    node = wall_localisation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()