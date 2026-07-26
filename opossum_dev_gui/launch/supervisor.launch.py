"""Lance le moniteur systeme + le superviseur de noeuds.

Ces deux noeuds alimentent la page « Systeme » de l'IHM web :
  * ``system_monitor``  -> /main_robot/system_stats  (CPU/RAM/temperature)
  * ``node_manager``    -> /main_robot/node_manager/{status,command}

Namespace par defaut : main_robot (surchargable via l'argument ``namespace``).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")

    declare_namespace = DeclareLaunchArgument(
        "namespace", default_value="main_robot",
        description="Namespace ROS du robot.",
    )

    node_manager_params = PathJoinSubstitution(
        [FindPackageShare("opossum_dev_gui"), "config", "node_manager.yaml"]
    )

    system_monitor = Node(
        namespace=namespace,
        package="opossum_dev_gui",
        executable="system_monitor.py",
        name="system_monitor",
        output="screen",
    )

    node_manager = Node(
        namespace=namespace,
        package="opossum_dev_gui",
        executable="node_manager.py",
        name="node_manager",
        output="screen",
        parameters=[node_manager_params],
    )

    return LaunchDescription([declare_namespace, system_monitor, node_manager])
