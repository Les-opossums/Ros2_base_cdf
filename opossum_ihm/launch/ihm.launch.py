"""Launch the nodes for simulation."""

from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction


def launch_setup(context, *args, **kwargs):
    """Launch setup."""
    nodes = []
    robot_names_str = LaunchConfiguration("robot_names").perform(context)
    robot_names_list = [name.strip() for name in robot_names_str.split(",")]

    param_file = PathJoinSubstitution(
        [FindPackageShare("opossum_ihm"), "config", "ihm_params.yaml"]
    )

    for robot in robot_names_list:
        node_ihm = Node(
            package="opossum_ihm",
            namespace=robot,
            executable="ihm_node.py",
            name="ihm_node",
            parameters=[param_file],
        )

        node_param_server = Node(
            package="opossum_ihm",
            namespace=robot,
            executable="parameters_server.py",
            name="param_server_node",
            parameters=[param_file],
        )
        nodes.append(node_ihm)
        nodes.append(node_param_server)
    return nodes


def generate_launch_description():
    """Generate launch description."""
    robot_names_arg = DeclareLaunchArgument(
        "robot_names",
        default_value="main_robot",  # For multiple names, e.g., "main_robot,other_robot,third_robot"
        description="Set all the simulated robots",
    )

    return LaunchDescription([robot_names_arg, OpaqueFunction(function=launch_setup)])
