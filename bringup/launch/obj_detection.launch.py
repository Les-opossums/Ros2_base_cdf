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

    node_obj_detector_haut = Node(
        package="obj_detector",
        namespace=namespace,
        executable="node_cluster",
        name="top_object_detector_node",
        parameters=[config],
    )

    ld.add_action(node_obj_detector_haut)

    return ld
