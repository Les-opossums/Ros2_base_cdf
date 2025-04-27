"""Launch the navigation node."""

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
        [FindPackageShare("opossum_nav"), "config", "nav_params.yaml"]
    )
    for robot in robot_names_list:
        node_avoid_obstacle = Node(
            namespace=robot,
            package="opossum_nav",
            executable="avoid_obstacle.py",
            name="avoid_obstacle_node",
            parameters=[param_file],
        )
        nodes.append(node_avoid_obstacle)
    return nodes


def generate_launch_description():
    """Generate the launch description."""

    robot_names_arg = DeclareLaunchArgument(
        "robot_names",
        default_value="main_robot",  # For multiple names, e.g., "main_robot,other_robot,third_robot"
        description="Set all the simulated robots",
    )
    simulation_arg = DeclareLaunchArgument(
        "simulation", default_value="false", description="Enable simulation mode"
    )

    return LaunchDescription(
        [robot_names_arg, simulation_arg, OpaqueFunction(function=launch_setup)]
    )
