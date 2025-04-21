"""Launch the nodes for communication."""

from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction


def launch_setup(context, *args, **kwargs):
    """Launch the setup."""
    nodes = []
    simulation = LaunchConfiguration("simulation").perform(context)
    simulation = simulation.lower() in ["true", "1"]
    robot_names_str = LaunchConfiguration("robot_names").perform(context)
    robot_names_list = [name.strip() for name in robot_names_str.split(",")]

    param_file = PathJoinSubstitution(
        [FindPackageShare("opossum_comm"), "config", "comm_params.yaml"]
    )

    for robot in robot_names_list:
        node_communication = Node(
            namespace=robot,
            package="opossum_comm",
            executable="comm.py",
            name="comm_node",
            parameters=[param_file, {"simulation": simulation}],
        )
        nodes.append(node_communication)
    return nodes


def generate_launch_description():
    """Generate launch description."""
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
