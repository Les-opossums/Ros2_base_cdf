"""Launch the obstacle extractor node standalone."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import yaml


def generate_launch_description():
    """Generate launch description for obstacle extractor."""
    param_file = os.path.join(
        get_package_share_directory("obstacle_detector"),
        "config",
        "obstacle_extractor.yaml",
    )

    node_obstacle_extractor = Node(
        package="obstacle_detector",
        executable="obstacle_extractor_node",
        name="obstacle_extractor_node",
        parameters=[param_file],
    )

    return LaunchDescription([node_obstacle_extractor])
