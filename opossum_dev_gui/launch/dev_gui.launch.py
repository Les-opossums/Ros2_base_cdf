"""Launching nodes for main."""

from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction


def launch_setup(context, *args, **kwargs):
    """Launch the setup."""
    nodes = []
    robot_names_str = LaunchConfiguration("robot_names").perform(context)
    robot_names_list = [name.strip() for name in robot_names_str.split(",")]

    param_file = PathJoinSubstitution(
        [FindPackageShare("opossum_dev_gui"), "config", "dev_gui_params.yaml"]
    )

    node_dev_gui = Node(
        package="opossum_dev_gui",
        executable="dev_gui.py",
        name="dev_gui_node",
        parameters=[param_file, {"robot_names": robot_names_list}],
    )
    nodes.append(node_dev_gui)
    return nodes


def generate_launch_description():
    """Generate launch description."""
    robot_names_arg = DeclareLaunchArgument(
        "robot_names",
        default_value="main_robot",  # For multiple names, e.g., "main_robot,other_robot,third_robot"
        description="Set all the simulated robots",
    )

    return LaunchDescription([robot_names_arg, OpaqueFunction(function=launch_setup)])
