"""Launch the IHM node."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description."""
    ld = LaunchDescription()

    namespace = "main_robot"

    node_ihm = Node(
        package="opossum_ihm",
        namespace=namespace,
        executable="ihm_node.py",
        name="node_ihm",
        parameters=[],
    )

    param_server = Node(
        package="opossum_ihm",
        namespace=namespace,
        executable="parameters_server.py",
        name="param_server",
        parameters=[{"team_color": "blue", "script_number": 1, "debug_mode": False}],
    )

    ld.add_action(node_ihm)
    ld.add_action(param_server)

    return ld
