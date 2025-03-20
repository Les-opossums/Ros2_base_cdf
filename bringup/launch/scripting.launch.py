import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    namespace = "main_robot"

    config = os.path.join(
        get_package_share_directory("bringup"),
        "config",
        "lidar_params.yaml",
    )

    node_action_sequencer = Node(
        package="action_sequencer",
        namespace=namespace,
        executable="action_sequencer_node",
        name="action_sequencer",
        parameters=[config],
    )

    node_big_brother = Node(
        package="big_brother",
        namespace=namespace,
        executable="big_brother_node",
        name="big_brother",
        parameters=[config],
    )

    ld.add_action(node_action_sequencer)
    ld.add_action(node_big_brother)

    return ld
