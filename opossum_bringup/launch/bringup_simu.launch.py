"""Core launcher robot."""
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """Create some nodes to be launched."""

    simulation_arg = DeclareLaunchArgument(
        "simulation", default_value="true", description="Enable simulation mode"
    )
    simulation = LaunchConfiguration("simulation")
    robot_names_arg = DeclareLaunchArgument(
        "robot_names", default_value="main_robot", description="Set list of robots"
    )
    robot_names = LaunchConfiguration("robot_names")

    simu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("opossum_simu"),
                        "launch",
                        "simu.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={"robot_names": robot_names}.items(),
    )
    localisation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("opossum_localisation"),
                        "launch",
                        "localisation.launch.py",
                    ]
                )
            ]
        ),
        # launch_arguments={'simulation': simulation}.items()
        launch_arguments={"simulation": simulation, "robot_names": robot_names}.items(),
    )

    dev_gui_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("opossum_dev_gui"), "launch", "dev_gui.launch.py"]
                )
            ]
        ),
        launch_arguments={"robot_names": robot_names}.items(),
    )

    comm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("opossum_comm"), "launch", "comm.launch.py"]
                )
            ]
        ),
        launch_arguments={"simulation": simulation, "robot_names": robot_names}.items()
        # launch_arguments={'simulation': simulation}.items()
    )

    return LaunchDescription(
        [
            simulation_arg,
            robot_names_arg,
            comm_launch,
            localisation_launch,
            simu_launch,
            dev_gui_launch,
        ]
    )
