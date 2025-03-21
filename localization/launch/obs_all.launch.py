"""Launch the beacon detector node and simulation for lidars."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import yaml


def generate_launch_description():
    """Generate launch description for all simulations."""
    param_file = os.path.join(
        get_package_share_directory("localization"),
        "config",
        "simu_config.yaml",
    )

    nodes = []
    with open(param_file, "r") as file:
        params = yaml.safe_load(file)

    top_keys = [key for key in params.keys() if not key.endswith("node")]
    node_position_sender = Node(
        package="localization",
        executable="position_sender",
        name="position_sender_node",
        parameters=[param_file, {"robot_names": top_keys}],
    )
    nodes.append(node_position_sender)

    node_orchestrator_gui = Node(
        package="orchestrator_gui",
        executable="orchestrator_gui_node.py",
        name="orchestrator_gui_node",
        parameters=[param_file, {"robot_names": top_keys}],
    )
    nodes.append(node_orchestrator_gui)

    node_tf_broadcaster = Node(
        package="localization",
        executable="tf_broadcaster.py",
        name="tf_broadcaster_node",
        parameters=[param_file, {"robot_names": top_keys}],
    )
    nodes.append(node_tf_broadcaster)

    for key in top_keys:
        node_lidar_simulation = Node(
            namespace=key,
            package="localization",
            executable="lidar_simulation.py",
            name="lidar_simulation_node",
            parameters=[param_file],
        )

        node_nav_simulation = Node(
            namespace=key,
            package="localization",
            executable="nav_simulation.py",
            name="nav_simulation_node",
            parameters=[param_file],
        )

        node_beacon_detector = Node(
            namespace=key,
            package="localization",
            executable="beacon_detector",
            name="beacon_detector_node",
            parameters=[param_file],
        )

        print(params[key]["lidar_simulation_node"]["ros__parameters"]["use_lidar_points"])
        if bool(params[key]["lidar_simulation_node"]["ros__parameters"]["use_lidar_points"]):
            node_obstacle_extractor = Node(
                namespace=key,
                package="obstacle_detector",
                executable="obstacle_extractor_node",
                name="obstacle_extractor_node",
                parameters=[param_file],
            )
            nodes.append(node_obstacle_extractor)
        nodes.append(node_lidar_simulation)
        nodes.append(node_nav_simulation)
        nodes.append(node_beacon_detector)
        
    return LaunchDescription(nodes)
