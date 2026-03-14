"""Core launcher robot."""
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    """Create some nodes to be launched."""

    simulation_arg = DeclareLaunchArgument(
        "simulation", default_value="false", description="Enable simulation mode"
    )
    simulation = LaunchConfiguration("simulation")
    robot_names_arg = DeclareLaunchArgument(
        "robot_names",
        default_value="main_robot",
        # default_value="main_robot,second_robot",
        description="Set list of robots",
    )
    robot_names = LaunchConfiguration("robot_names")

    dev_gui_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("opossum_dev_gui"), "launch", "dev_gui.launch.py"]
                )
            ]
        ),
        launch_arguments={"robot_names": robot_names, "simulation": simulation}.items(),
    )

    return LaunchDescription(
        [
            simulation_arg,
            robot_names_arg,
            dev_gui_launch,
        ]
    )
