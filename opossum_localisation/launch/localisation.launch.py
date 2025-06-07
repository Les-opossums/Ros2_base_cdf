"""Launch the beacon detector node and simulation for lidars."""

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
        [FindPackageShare("opossum_localisation"), "config", "localisation_params.yaml"]
    )

    for robot in robot_names_list:
        node_tf_broadcaster = Node(
            namespace=robot,
            package="opossum_localisation",
            executable="tf_broadcaster.py",
            name="tf_broadcaster_node",
            parameters=[param_file, {"robot_names": robot_names_list}],
        )
        node_beacon_detector = Node(
            namespace=robot,
            package="opossum_localisation",
            executable="beacon_detector",
            name="beacon_detector_node",
            parameters=[param_file],
        )

        node_obstacle_extractor = Node(
            namespace=robot,
            package="obstacle_detector",
            executable="obstacle_extractor_node",
            name="obstacle_extractor_node",
            parameters=[param_file],
        )

        if not simulation:
            node_rplidar = Node(
                package="rplidar_ros",
                namespace=robot,
                executable="rplidar_node",
                parameters=[param_file],
            )
            nodes.append(node_rplidar)
        nodes.append(node_obstacle_extractor)
        nodes.append(node_beacon_detector)
        nodes.append(node_tf_broadcaster)
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
