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
        get_package_share_directory("orchestrator_gui"),
        "config",
        "gui_config.yaml",
    )

    node_orchestrator_gui = Node(
        package="orchestrator_gui",
        namespace=namespace,
        executable="orchestrator_gui_node.py",
        name="orchestrator_gui_node",
        parameters=[config],
    )

    ld.add_action(node_orchestrator_gui)

    print(LaunchIntrospector().format_launch_description(ld))
    return ld
