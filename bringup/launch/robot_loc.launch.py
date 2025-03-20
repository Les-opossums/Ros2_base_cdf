import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch import LaunchIntrospector  # noqa: E402
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    namespace = "main_robot"

    config = os.path.join(
        get_package_share_directory("bringup"),
        "config",
        "lidar_params.yaml",
    )

    node_beacon_detector = Node(
        package="beacon_detector",
        namespace=namespace,
        executable="beacon_detector_node",
        name="beacon_detector_node",
        parameters=[config],
    )

    node_robot_data_merger = Node(
        package="robot_data_merger",
        namespace=namespace,
        executable="robot_data_merger_node",
        name="robot_data_merger_node",
        parameters=[config],
    )

    ld.add_action(node_beacon_detector)
    ld.add_action(node_robot_data_merger)

    return ld
