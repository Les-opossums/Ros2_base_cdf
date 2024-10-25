import os
from ament_index_python.packages import get_package_share_directory  # type: ignore
from launch import LaunchDescription
from launch import LaunchIntrospector  # noqa: E402
from launch_ros.actions import Node  # type: ignore


def generate_launch_description():
    ld = LaunchDescription()

    namespace = "main_robot"

    config = os.path.join(
        get_package_share_directory("bringup"),
        "config",
        "simu_params.yaml",
    )

    node_beacon_generator = Node(
        package="beacon_detector",
        namespace=namespace,
        executable="beacon_generator_node",
        name="beacon_generator_node",
        parameters=[config],
    )

    node_beacon_detector_top = Node(
        package="beacon_detector",
        namespace=namespace,
        executable="beacon_detector_node",
        name="beacon_detector_node_top",
        parameters=[config],
    )

    node_beacon_detector_bottom = Node(
        package="beacon_detector",
        namespace=namespace,
        executable="beacon_detector_node",
        name="beacon_detector_node_bottom",
        parameters=[config],
    )

    node_robot_data_merger = Node(
        package="robot_data_merger",
        namespace=namespace,
        executable="robot_data_merger_node",
        name="robot_data_merger_node",
        parameters=[config],
    )

    node_avoid_obstacle = Node(
        package="avoid_obstacle",
        namespace=namespace,
        executable="avoid_obstacle_node",
        name="avoid_obstacle_node",
        parameters=[config],
    )

    node_zc_detector = Node(
        package="beacon_detector",
        namespace=namespace,
        executable="beacon_detector_node",
        name="zc_detector_node",
        parameters=[config],
    )

    ld.add_action(node_beacon_generator)
    ld.add_action(node_beacon_detector_top)
    ld.add_action(node_beacon_detector_bottom)
    ld.add_action(node_robot_data_merger)
    ld.add_action(node_avoid_obstacle)
    ld.add_action(node_zc_detector)

    print(LaunchIntrospector().format_launch_description(ld))
    return ld
