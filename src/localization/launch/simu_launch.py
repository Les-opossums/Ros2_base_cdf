import os
from ament_index_python.packages import get_package_share_directory  # type: ignore
from launch import LaunchDescription
from launch import LaunchIntrospector  # noqa: E402
from launch_ros.actions import Node  # type: ignore
import yaml


def generate_launch_description():
    ld = LaunchDescription()

    namespace = "main_robot"

    config = os.path.join(
        get_package_share_directory("localization"),
        "config",
        "simu_config.yaml",
    )

    node_lidar_simulation = Node(
        package="localization",
        namespace=namespace,
        executable="lidar_simulation.py",
        name="lidar_simulation_node",
        parameters=[config],
    )

    node_nav_simulation = Node(
        package="localization",
        namespace=namespace,
        executable="nav_simulation.py",
        name="nav_simulation_node",
        parameters=[config],
    )

    node_beacon_detector = Node(
        package="localization",
        namespace=namespace,
        executable="beacon_detector.py",
        name="beacon_detector_node",
        parameters=[config],
    )

    ld.add_action(node_lidar_simulation)
    ld.add_action(node_nav_simulation)
    ld.add_action(node_beacon_detector)

    print(LaunchIntrospector().format_launch_description(ld))
    return ld
