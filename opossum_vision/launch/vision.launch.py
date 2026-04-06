"""Launch the vision system for multiple cameras and robots."""

import os
from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction

def launch_setup(context, *args, **kwargs):
    nodes = []

    # 1. Récupération des noms de robots depuis l'argument
    robot_names_str = LaunchConfiguration("robot_names").perform(context)
    robot_names_list = [name.strip() for name in robot_names_str.split(",") if name.strip()]

    # 2. Chemin vers le fichier YAML de configuration
    param_file = PathJoinSubstitution(
        [FindPackageShare("opossum_vision"), "config", "vision_params.yaml"]
    )

    # 3. Noms des caméras à lancer par robot
    # Ils détermineront le nom du noeud et iront chercher le bloc correspondant dans le YAML
    cameras = ["one", "two", "three"]

    # 4. Lancement des nœuds par robot et par caméra
    for robot in robot_names_list:
        for cam in cameras:
            # Le nom sera "vision_node_left", "vision_node_right", "vision_node_back"
            node_name = f"vision_node_{cam}" 
            
            node_vision = Node(
                package="opossum_vision",
                executable="vision_node.py", # Nom tel que défini dans setup.py
                name=node_name,              # Crucial : Doit matcher la clé du YAML
                namespace=robot,             # Ex: /main_robot
                output="screen",
                parameters=[param_file],     # Charge le YAML complet
                # Optionnel: Remapping si on veut tout forcer dans un seul topic
                # remappings=[('aruco_loc', '/aruco_loc_global')] 
            )
            nodes.append(node_vision)

    return nodes

def generate_launch_description():
    
    robot_names_arg = DeclareLaunchArgument(
        "robot_names",
        default_value="main_robot",
        description="Liste des robots séparés par des virgules",
    )

    return LaunchDescription([
        robot_names_arg,
        OpaqueFunction(function=launch_setup)
    ])