"""Launching nodes for main."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import yaml


def generate_launch_description():
    """Generate the launch description."""
    param_file = os.path.join(
        get_package_share_directory("bringup"),
        "config",
        "lidar_cfg.yaml",
    )

    nodes = []
    with open(param_file, "r") as file:
        params = yaml.safe_load(file)

    top_keys = [key for key in params.keys() if not key.endswith("node")]

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

        node_rplidar = Node(
            package="rplidar_ros",
            namespace=key,
            executable="rplidar_node",
            name="rp_lidar",
            parameters=[
                {
                    "serial_port": "/dev/ttyUSB1",
                    "serial_baudrate": 256000,
                    "channel_type": "serial",
                    "frame_id": "laser",
                    "inverted": False,
                    "angle_compensate": True,
                    "scan_mode": "Sensitivity",
                }
            ],
        )

        node_beacon_detector = Node(
            namespace=key,
            package="localization",
            executable="beacon_detector",
            name="beacon_detector_node",
            parameters=[param_file],
        )

        node_obstacle_extractor = Node(
            namespace=key,
            package="obstacle_detector",
            executable="obstacle_extractor_node",
            name="obstacle_extractor_node",
            parameters=[param_file],
        )
        nodes.append(node_obstacle_extractor)
        nodes.append(node_beacon_detector)
        nodes.append(node_rplidar)

    return LaunchDescription(nodes)
