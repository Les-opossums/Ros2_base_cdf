"""Launch the vision system and emulator."""

import os
from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction


def launch_setup(context, *args, **kwargs):
    nodes = []

    # 1. Récupération des noms de robots (argument ligne de commande)
    robot_names_str = LaunchConfiguration("robot_names").perform(context)
    robot_names_list = [name.strip() for name in robot_names_str.split(",") if name.strip()]

    # 2. Chemin vers le fichier YAML
    # Remplacez 'opossum_vision' par le nom exact de votre package
    param_file = PathJoinSubstitution(
        [FindPackageShare("opossum_vision"), "config", "vision_params.yaml"]
    )

    # 3. Lancement des nœuds par robot (Vision)
    for robot in robot_names_list:
        node_vision = Node(
            package="opossum_vision",
            executable="vision_node.py", # Nom défini dans setup.py
            name="vision_node",       # Le nom du noeud (doit matcher le YAML)
            namespace=robot,          # EX: main_robot
            output="screen",
            parameters=[param_file],  # Charge les params depuis le YAML
            # On ajoute un remapping si besoin pour être sûr des topics
            # remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')] 
        )
        nodes.append(node_vision)

    # 4. Lancement de l'Émulateur (Unique, global)
    # L'émulateur n'est pas lié à un robot spécifique dans cet exemple,
    # il simule le hardware connecté au PC.
    for robot in robot_names_list:
        node_emulator = Node(
            package="opossum_vision",
            executable="vision_emulator_node.py",
            name="vision_emulator_node",
            namespace=robot,
            output="screen",
            parameters=[param_file] # Il ira chercher les params globaux (/**)
        )
        nodes.append(node_emulator)

    return nodes


def generate_launch_description():
    
    robot_names_arg = DeclareLaunchArgument(
        "robot_names",
        #default_value="main_robot, second_robot", # Valeur par défaut correspondant au YAML
        default_value="main_robot",
        description="Liste des robots séparés par des virgules",
    )

    return LaunchDescription([
        robot_names_arg,
        OpaqueFunction(function=launch_setup)
    ])