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

    node_uart_com = Node(
        package="uart_com",
        namespace=namespace,
        executable="uart_com_node",
        name="uart_com",
        parameters=[config]
    )

    node_ihm = Node(
        package="ihm_ros",
        namespace=namespace,
        executable="ihm_ros_node",
        name="ihm_ros",
        parameters=[config]
    )

    node_avoid_obstacle = Node(
        package="avoid_obstacle",
        namespace=namespace,
        executable="avoid_obstacle_node",
        name="avoid_obstacle_node",
        parameters=[config],
    )

    ld.add_action(node_ihm)
    ld.add_action(node_uart_com)
    ld.add_action(node_avoid_obstacle)

    return ld
