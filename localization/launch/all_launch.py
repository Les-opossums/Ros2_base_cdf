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
    print(top_keys)
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

    for key in top_keys:
        node_lidar_simulation = Node(
            namespace=key,
            package="localization",
            executable="lidar_simulation.py",
            name="lidar_simulation_node",
            parameters=[param_file],
        )

        node_beacon_detector = Node(
            namespace=key,
            package="localization",
            executable="beacon_detector",
            name="beacon_detector_node",
            parameters=[param_file],
        )
        nodes.append(node_lidar_simulation)
        nodes.append(node_beacon_detector)
    return LaunchDescription(nodes)
