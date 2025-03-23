import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch import LaunchIntrospector
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory("opossum_simu"),
        "config",
        "simu_comm.yaml",
    )

    node_mini_action_requester = Node(
        package="opossum_simu",
        executable="mini_action_requester.py",
        name="mini_action_requester_node",
        parameters=[config],
    )

    node_zynq_simu = Node(
        package="opossum_simu",
        executable="zynq_simu.py",
        name="zynq_simu_node",
        parameters=[config],
    )

    ld.add_action(node_mini_action_requester)
    ld.add_action(node_zynq_simu)

    print(LaunchIntrospector().format_launch_description(ld))
    return ld
