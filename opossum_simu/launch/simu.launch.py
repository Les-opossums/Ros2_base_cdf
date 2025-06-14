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
        [FindPackageShare("opossum_simu"), "config", "simu_params.yaml"]
    )

    node_position_sender = Node(
        package="opossum_simu",
        executable="position_sender.py",
        name="position_sender_node",
        parameters=[param_file, {"robot_names": robot_names_list}],
    )
    nodes.append(node_position_sender)

    for robot in robot_names_list:
        node_zynq_simu = Node(
            namespace=robot,
            package="opossum_simu",
            executable="zynq_simu.py",
            name="zynq_simu_node",
            parameters=[param_file],
        )

        node_motor_simu = Node(
            namespace=robot,
            package="opossum_simu",
            executable="motor_simu.py",
            name="motor_simu_node",
            parameters=[param_file],
        )

        node_lidar_simu = Node(
            namespace=robot,
            package="opossum_simu",
            executable="lidar_simu.py",
            name="lidar_simu_node",
            parameters=[param_file],
        )
        nodes.append(node_motor_simu)
        nodes.append(node_zynq_simu)
        nodes.append(node_lidar_simu)
    return nodes


def generate_launch_description():
    """Generate launch description."""
    robot_names_arg = DeclareLaunchArgument(
        "robot_names",
        default_value="main_robot",  # For multiple names, e.g., "main_robot,other_robot,third_robot"
        description="Set all the simulated robots",
    )

    return LaunchDescription([robot_names_arg, OpaqueFunction(function=launch_setup)])
