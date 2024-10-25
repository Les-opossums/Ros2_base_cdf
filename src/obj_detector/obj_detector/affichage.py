import rclpy
from rclpy.node import Node

from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point


def affichage_plante(coordonnees,radius):
    marker_array = MarkerArray()
    
    for i in range(len(coordonnees)):
        marker = Marker()
        marker.header.frame_id = 'laser'
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.scale.x = radius[i]*2
        marker.scale.y = radius[i]*2
        marker.scale.z = radius[i]*2
        marker.color.a = 1.0  # Opacity
        marker.color.r = 0.0  # Red
        marker.color.g = 1.0  # Green
        marker.color.b = 0.0  # Blue
        marker.pose.orientation.w = 1.0  # No rotation
        marker.pose.position.x = coordonnees[i][0]
        marker.pose.position.y = coordonnees[i][1]
        marker.pose.position.z = 0.0  # You may need to adjust this value
        marker.id = i  # Utilisez l'indice de la boucle comme identifiant unique
        
        marker_array.markers.append(marker)

    return marker_array

def affichage_segment(coordonnees):
    marker_array = MarkerArray()

    for i, obstacle in enumerate(coordonnees):
        # Créer un marqueur de type LINE_LIST
        marker = Marker()
        marker.header.frame_id = 'laser'
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.01  # Épaisseur de la ligne
        marker.color.a = 1.0  # Opacity
        marker.color.r = 0.0  # Red
        marker.color.g = 0.0  # Green
        marker.color.b = 1.0  # Blue
        marker.pose.orientation.w = 1.0  # Pas de rotation
        marker.id = i  # Identifiant unique

        # Ajouter les points de début et de fin de la ligne
        point_debut = Point()
        point_debut.x = obstacle[0][0]
        point_debut.y = obstacle[0][1]
        point_debut.z = 0.0  # Ajustez si nécessaire

        point_fin = Point()
        point_fin.x = obstacle[1][0]
        point_fin.y = obstacle[1][1]
        point_fin.z = 0.0  # Ajustez si nécessaire

        # Ajouter les points de début et de fin à la liste de points
        marker.points.append(point_debut)
        marker.points.append(point_fin)

        # Ajouter le marqueur à la liste des marqueurs
        marker_array.markers.append(marker)

    return marker_array

def affichage_point(coordonnees):
    marker_array = MarkerArray()
    
    for i in range(len(coordonnees)):
        marker = Marker()
        marker.header.frame_id = 'laser'
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.scale.x = 0.005
        marker.scale.y = 0.005
        marker.scale.z = 0.005
        marker.color.a = 1.0  # Opacity
        marker.color.r = 1.0  # Red
        marker.color.g = 0.0  # Green
        marker.color.b = 0.0  # Blue
        marker.pose.orientation.w = 1.0  # No rotation
        marker.pose.position.x = coordonnees[i][0]
        marker.pose.position.y = coordonnees[i][1]
        marker.pose.position.z = 0.0  # You may need to adjust this value
        marker.id = i  # Utilisez l'indice de la boucle comme identifiant unique
        
        marker_array.markers.append(marker)

    return marker_array

def affichage_point_debug(coordonnees):
    marker_array = MarkerArray()
    
    for i in range(len(coordonnees)):
        marker = Marker()
        marker.header.frame_id = 'laser'
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.a = 1.0  # Opacity
        marker.color.r = 1.0  # Red
        marker.color.g = 1.0  # Green
        marker.color.b = 0.0  # Blue
        marker.pose.orientation.w = 1.0  # No rotation
        marker.pose.position.x = coordonnees[i][0]
        marker.pose.position.y = coordonnees[i][1]
        marker.pose.position.z = 0.0  # You may need to adjust this value
        marker.id = i  # Utilisez l'indice de la boucle comme identifiant unique
        
        marker_array.markers.append(marker)

    return marker_array