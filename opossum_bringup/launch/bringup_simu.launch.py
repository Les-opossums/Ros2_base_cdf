"""Core launcher robot."""
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    """Create some nodes to be launched."""

    simulation_arg = DeclareLaunchArgument(
        "simulation", default_value="true", description="Enable simulation mode"
    )
    simulation = LaunchConfiguration("simulation")

    robot_names_arg = DeclareLaunchArgument(
        "robot_names",
        # default_value="main_robot",
        default_value="main_robot,second_robot",
        description="Set list of robots",
    )
    robot_names = LaunchConfiguration("robot_names")

    board_config_arg = DeclareLaunchArgument(
        "board_config",
        default_value="objects",
        description="Set list of robots",
    )
    board_config = LaunchConfiguration("board_config")

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
        launch_arguments={"robot_names": robot_names, "board_config": board_config}.items(),
    )

    nav_params_arg = DeclareLaunchArgument(
        "nav_params_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("opossum_nav"), "config", "nav_params.yaml"]
        ),
        description="Chemin complet vers le fichier YAML des paramètres de navigation",
    )
    nav_params_file = LaunchConfiguration("nav_params_file")

    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("opossum_nav"),
                        "launch",
                        "nav.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "robot_names": robot_names, 
            "nav_params_file": nav_params_file
            }.items(),
    )

    localisation_params_arg = DeclareLaunchArgument(
        "localisation_params_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("opossum_localisation"), "config", "localisation_params.yaml"]
        ),
        description="Chemin complet vers le fichier YAML de localisation",
    )
    localisation_params_file = LaunchConfiguration("localisation_params_file")

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
        launch_arguments={"simulation": simulation, 
                          "robot_names": robot_names,
                          "localisation_params_file": localisation_params_file
                          }.items(),
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

    ihm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("opossum_ihm"), "launch", "ihm.launch.py"]
                )
            ]
        ),
        launch_arguments={"simulation": simulation, "robot_names": robot_names}.items()
    )

    action_sequencer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("opossum_action_sequencer"), "launch", "action_sequencer.launch.py"]
                )
            ]
        ),
        launch_arguments={"robot_names": robot_names, "board_config": board_config}.items()
    )

    return LaunchDescription(
        [
            simulation_arg,
            robot_names_arg,
            board_config_arg,
            nav_params_arg,
            localisation_params_arg,
            comm_launch,
            nav_launch,
            localisation_launch,
            simu_launch,
            dev_gui_launch,
            ihm_launch,
            action_sequencer_launch,
        ]
    )
