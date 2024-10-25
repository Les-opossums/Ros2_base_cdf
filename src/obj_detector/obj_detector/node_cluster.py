import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import MarkerArray, Marker
import numpy as np
import obj_detector.affichage as affichage
import obj_detector.trigo as trigo 
from geometry_msgs.msg import Point
from cdf_msgs.msg import Obstacles, CircleObstacle, SegmentObstacle
#import time 

class node_cluster(Node):
    def __init__(self):
        super().__init__('node_cluster')
        self._init_parameters()
        self._init_publishers() 
        self._init_subscribers()
        self.place = self.get_parameter("place").get_parameter_value().string_value

        if self.place == "haut":
            self.bon_point = list(range(1798))
            self.angle_correction = -np.pi/2
            self.rotation_correction = 1
            self.marge = 0.15
            self.longueur_cercle = 0.05
            self.max_distance = 3.8
        
        elif self.place == "bas":
            self.bon_point = list(range(170, 420)) + list(range(770, 1055)) + list(range(1400, 1660))
            self.angle_correction = 2*np.pi/3 #-2*np.pi/3 
            self.rotation_correction = -1
            self.marge = 0.06
            self.longueur_cercle = 0.025
            self.max_distance = 0.5

        elif self.place == "zone_calcul":
            self.bon_point = list(range(0,500)) + list(range(1300,1798))
            self.angle_correction = 0
            self.rotation_correction = 1
            self.marge = 0.15
            self.longueur_cercle = 0.05
            self.max_distance = 3.

        else :
            self.get_logger().warn(f'Mode debug activé.')
            #self.bon_point = list(range(170, 420)) + list(range(770, 1055)) + list(range(1400, 1660))
            self.bon_point = list(range(1798))
            self.angle_correction = 0
            self.rotation_correction = 1
            self.marge = 0.06
            self.longueur_cercle = 0.05
            self.max_distance = 5

    def _init_parameters(self):
        self.declare_parameters(
            namespace="",
            parameters=[
                ("scan_topic", "/scan"),
                ("object_topic", "obstacle"),
                ("object_display_topic", "object_display"),
                ("place", "debug")
            ],
        )
        #self.get_logger().info(f'Parameters: {self.get_parameters(["scan_topic", "object_topic", "object_display_topic", "place"])}')
        self.get_logger().info(f'Place: {self.get_parameter("place").get_parameter_value().string_value}')

    def _init_publishers(self):
        self.object_display_topic = self.get_parameter("object_display_topic").get_parameter_value().string_value
        self.publisher_ = self.create_publisher(
            MarkerArray, 
            self.object_display_topic,
            10)

        self.object_topic = self.get_parameter("object_topic").get_parameter_value().string_value
        self.publisher_obstacle = self.create_publisher(
            Obstacles,
            self.object_topic,
            10)

    def _init_subscribers(self):
        self.scan_topic = self.get_parameter("scan_topic").get_parameter_value().string_value
        self.subscription = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.obstacle_detect_callback,
            1)
        self.subscription

    def coordonnee_point(self, i, msg : LaserScan):
        theta_min = msg.angle_min + self.angle_correction
        delta_theta = msg.angle_increment
        return [np.cos(self.rotation_correction*(i*delta_theta)+theta_min)*float(msg.ranges[i]), 
                np.sin(self.rotation_correction*(i*delta_theta)+theta_min)*float(msg.ranges[i])]

    def coordonnee_cercle(self, segment, msg:LaserScan):
        theta_min = msg.angle_min + self.angle_correction
        delta_theta = msg.angle_increment

        point_milieu = int((segment[1]-segment[0])/2)+segment[0]
        dist = msg.ranges[point_milieu]+np.sqrt(3)*self.longueur_cercle/2
        return [np.cos(self.rotation_correction*(point_milieu*delta_theta)+theta_min)*dist,
                np.sin(self.rotation_correction*(point_milieu*delta_theta)+theta_min)*dist]

    
    def distance(self, i, j, msg : LaserScan):
        return np.sqrt(float(msg.ranges[i])**2 + float(msg.ranges[j])**2 - 2*float(msg.ranges[i])*float(msg.ranges[j])*np.cos(np.abs((j-i))*float(msg.angle_increment)))
    
    def sortie_obstacle(self, liste_obstacle, msg : LaserScan):
        if len(liste_obstacle) == 0:
            return None
        else :
            obstacle = [liste_obstacle[0], liste_obstacle[-1]]
            return obstacle
        
    def traitement_segment(self, segment, msg:LaserScan):
        nb_point = segment[1]-segment[0]
        delta_theta = msg.angle_increment

        seuil_angle = 0.05
    
        if nb_point < 10 :
            return [segment]
        
        else :
            liste_segment = []
            point_milieu = segment[0]+10

            for k in range(int(nb_point/10)-1):
                cos_alpha_1 = trigo.cos_alpha([segment[0],point_milieu],msg)
                cos_alpha_2 = trigo.cos_alpha([segment[0],point_milieu+10],msg)
                if np.abs(cos_alpha_1 - cos_alpha_2) < seuil_angle :
                    point_milieu = point_milieu + 10

                else :
                    liste_segment.append([segment[0],point_milieu-10])
                    segment[0] = point_milieu
                    point_milieu = point_milieu+10
            
            cos_alpha_1 = trigo.cos_alpha([segment[0],point_milieu],msg)
            cos_alpha_2 = trigo.cos_alpha([segment[0],segment[1]],msg)
            if np.abs(cos_alpha_1 - cos_alpha_2) < seuil_angle :
                liste_segment.append([segment[0],segment[1]])
            else :
                liste_segment.append([segment[0],point_milieu-10])
                liste_segment.append([point_milieu,segment[1]])
            return liste_segment

    def obstacle_detect_callback(self, msg: LaserScan):
        #start = time.time()
        number_of_points = len(msg.ranges)
        theta_min = msg.angle_min
        delta_theta = msg.angle_increment

        liste_obstacles = []
        points_obstacles = []

        coordonnee_obstacle = []
        coordonnee_plante = []
        radius_plante = []
        theta_plante = []
        index_plante = []
        coordonnee_segment = [] 
        taille_obstacle = []
        coordonnee_mesure = []
        coordonnee_mesure_debug = []
        liste_segment = []
        
        for k in self.bon_point:
            try :
                if str(msg.ranges[k]) == 'inf' or str(msg.ranges[k+1]) == 'inf' or float(msg.ranges[k]) > self.max_distance or float(msg.ranges[k+1]) > self.max_distance:
                    liste_obstacles.append(self.sortie_obstacle(points_obstacles, msg))
                    points_obstacles = []
                    #self.get_logger().warn(f'Value not valid: {msg.ranges[k]} or {msg.ranges[k+1]}.')

                elif k+1 not in self.bon_point:
                    liste_obstacles.append(self.sortie_obstacle(points_obstacles, msg))
                    points_obstacles = []

                elif np.abs(float(msg.ranges[k]) - float(msg.ranges[k+1])) < 0.05:
                    points_obstacles.append(k)

                else : 
                    coordonnee_mesure.append(self.coordonnee_point(k, msg))
                    liste_obstacles.append(self.sortie_obstacle(points_obstacles, msg))
                    points_obstacles = []
            except:
                self.get_logger().warn(f'{number_of_points} points dans le scan.')
        
        liste_obstacles.append(self.sortie_obstacle(points_obstacles, msg))

        new_liste_obstacles = [x for x in liste_obstacles if x != None]
        liste_obstacles = new_liste_obstacles

        for i in range(len(liste_obstacles)):
            taille_obstacle = self.distance(liste_obstacles[i][0], liste_obstacles[i][1], msg)
            if taille_obstacle < self.marge :
                coordonnee_plante.append(self.coordonnee_cercle(liste_obstacles[i], msg))
                radius_plante.append(taille_obstacle*np.sqrt(3)/3)
                point_milieu = int((liste_obstacles[i][1]-liste_obstacles[i][0])/2)+liste_obstacles[i][0]
                theta_plante.append((self.rotation_correction*(point_milieu*delta_theta)+theta_min)%(2*np.pi)+self.angle_correction)
                index_plante.append(point_milieu) 
                #if self.place == "bas":
                #    self.get_logger().info(f'Angle {(self.rotation_correction*(point_milieu*delta_theta)+theta_min)%(2*np.pi)-np.pi/3}')
                #    self.get_logger().info(f'Plante détectée: {coordonnee_plante[-1]}')
                #    self.get_logger().info(f'Distance: {msg.ranges[int((liste_obstacles[i][1]-liste_obstacles[i][0])/2)+liste_obstacles[i][0]]}')
                #    self.get_logger().info(f'Angle: {int((liste_obstacles[i][1]-liste_obstacles[i][0])/2)+liste_obstacles[i][0]}')
                #    self.get_logger().info(f'Radius: {radius_plante[-1]}')

            else :
                segment_temp = self.traitement_segment(liste_obstacles[i],msg)
                for k in range(len(segment_temp)):
                    coordonnee_segment.append([self.coordonnee_point(segment_temp[k][0], msg), 
                                              self.coordonnee_point(segment_temp[k][1], msg)])
                    liste_segment.append(segment_temp[k])
                
        marker_array_circle = affichage.affichage_plante(coordonnee_plante, radius_plante)
        self.publisher_.publish(marker_array_circle)

        marker_array_segment = affichage.affichage_segment(coordonnee_segment)
        self.publisher_.publish(marker_array_segment)

        #marker_array_mesure = affichage.affichage_point(coordonnee_mesure)
        #self.publisher_.publish(marker_array_mesure)

        #marker_array_mesure = affichage.affichage_point_debug(coordonnee_mesure_debug)
        #marker_array_mesure = affichage.affichage_point_debug([[0.,0.]])
        #self.publisher_.publish(marker_array_mesure)        

        obstacle = Obstacles()
        for i in range(len(coordonnee_plante)):
            circle = CircleObstacle()
            circle.center.x = coordonnee_plante[i][0]
            circle.center.y = coordonnee_plante[i][1]
            #if self.place == "bas":
            #    self.get_logger().info(f'Plante détectée: {coordonnee_plante[i]}')
            circle.radius = radius_plante[i]
            circle.theta = theta_plante[i]
            circle.index_point = index_plante[i]
            obstacle.circles.append(circle)

        for i in range(len(coordonnee_segment)):
            segment = SegmentObstacle()
            segment.first_point.x = coordonnee_segment[i][0][0]
            segment.first_point.y = coordonnee_segment[i][0][1]
            segment.last_point.x = coordonnee_segment[i][1][0]
            segment.last_point.y = coordonnee_segment[i][1][1]
            segment.index_first_point = liste_segment[i][0]
            segment.index_last_point = liste_segment[i][1]
            obstacle.segments.append(segment)
        self.publisher_obstacle.publish(obstacle)
    
        #end = time.time()
        #self.get_logger().info(f'Time elapsed: {end-start} s.')

def main():
    print('Hi from plante_detector.')
    rclpy.init(args=None)
    node = node_cluster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()