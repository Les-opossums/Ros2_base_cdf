"""Launch the nodes for communication."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import yaml


def generate_launch_description():
    """Launch the nodes for communication."""
    param_file = os.path.join(
        get_package_share_directory("opossum_comm"),
        "config",
        "comm_params.yaml",
    )

    nodes = []
    with open(param_file, "r") as file:
        params = yaml.safe_load(file)

    top_keys = [key for key in params.keys() if not key.endswith("node")]
    print(top_keys)

    for key in top_keys:
        node_communication = Node(
            namespace=key,
            package="opossum_comm",
            executable="comm.py",
            name="communication_node",
            parameters=[param_file],
        )
        nodes.append(node_communication)
    return LaunchDescription(nodes)
