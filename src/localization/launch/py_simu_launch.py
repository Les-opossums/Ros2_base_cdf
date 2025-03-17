import os
from ament_index_python.packages import get_package_share_directory  # type: ignore
from launch import LaunchDescription
from launch import LaunchIntrospector  # noqa: E402
from launch_ros.actions import Node  # type: ignore


def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory("localization"),
        "config",
        "simu_config.yaml",
    )

    node_lidar_simulation = Node(
        package="localization",
        executable="lidar_simulation.py",
        name="lidar_simulation_node",
        parameters=[config],
    )

    node_nav_simulation = Node(
        package="localization",
        executable="nav_simulation.py",
        name="nav_simulation_node",
        parameters=[config],
    )

    node_position_sender = Node(
        package="localization",
        executable="position_sender.py",
        name="position_sender_node",
        parameters=[config],
    )

    node_beacon_detector = Node(
        package="localization",
        executable="beacon_detector.py",
        name="beacon_detector_node",
        parameters=[config],
    )

    ld.add_action(node_lidar_simulation)
    ld.add_action(node_nav_simulation)
    ld.add_action(node_beacon_detector)
    ld.add_action(node_position_sender)

    print(LaunchIntrospector().format_launch_description(ld))
    return ld
