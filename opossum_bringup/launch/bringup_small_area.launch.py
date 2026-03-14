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
        default_value="main_robot",
        # default_value="main_robot,second_robot",
        description="Set list of robots",
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

    nav_params_arg = DeclareLaunchArgument(
        "nav_params_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("opossum_nav"), "config", "nav_small_area_params.yaml"]
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
            [FindPackageShare("opossum_localisation"), "config", "localisation_small_area_params.yaml"]
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
        launch_arguments={"robot_names": robot_names}.items()
    )

    return LaunchDescription(
        [
            simulation_arg,
            robot_names_arg,
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

"""Launch the IHM node."""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterFile


def generate_launch_description():
    """Generate launch description."""
    ld = LaunchDescription()

    namespace = "main_robot"

    node_ihm = Node(
        package="opossum_ihm",
        namespace=namespace,
        executable="ihm_node.py",
        name="node_ihm",
        parameters=[],
    )

    param_server = Node(
        package="opossum_ihm",
        namespace=namespace,
        executable="parameters_server.py",
        name="param_server",
        parameters=[{"team_color": "blue",
                     "script_number": 1,
                     "debug_mode": False}],
    )

    node_action_sequencer = Node(
        package="opossum_action_sequencer",
        namespace=namespace,
        executable="action_sequencer_node.py",
        name="action_sequencer_node",
        parameters=[{}],
    )

    param_com_path = PathJoinSubstitution(
        [FindPackageShare("opossum_comm"),
         "config",
         "comm_params.yaml"]
    )
    param_com = ParameterFile(param_com_path, allow_substs=True)

    node_com = Node(
        package="opossum_comm",
        namespace=namespace,
        executable="comm.py",
        name="comm_node",
        parameters=[param_com, {"simulation": False}],
    )

    param_loc_path = PathJoinSubstitution(
        [FindPackageShare("opossum_localisation"),
         "config",
         "localisation_small_area_params.yaml"]
    )
    param_loc = ParameterFile(param_loc_path, allow_substs=True)

    node_tf_broadcaster = Node(
        package="opossum_localisation",
        executable="tf_broadcaster.py",
        name="tf_broadcaster_node",
        parameters=[param_loc, {"robot_names": ["main_robot"]}],
        respawn=True,
        respawn_delay=2.0,
        respawn_max_retries=1.0,
    )

    node_rplidar = Node(
        package="rplidar_ros",
        namespace=namespace,
        executable="rplidar_node",
        name="rplidar_node",
        parameters=[param_loc],
    )

    node_beacon_detector = Node(
        namespace=namespace,
        package="opossum_localisation",
        executable="beacon_detector",
        name="beacon_detector_node",
        parameters=[param_loc],
    )

    node_obstacle_extractor = Node(
        namespace=namespace,
        package="obstacle_detector",
        executable="obstacle_extractor_node",
        name="obstacle_extractor_node",
        parameters=[param_loc],
    )

    param_nav = PathJoinSubstitution(
        [FindPackageShare("opossum_nav"), "config", "nav_small_area_params.yaml"]
    )
    
    node_avoid_obstacle = Node(
        namespace=namespace,
        package="opossum_nav",
        executable="avoid_obstacle.py",
        name="avoid_obstacle_node",
        parameters=[param_nav],
        )

    param_vision_path = PathJoinSubstitution(
        [FindPackageShare("opossum_vision"), 
        "config", 
        "vision_params.yaml"]
    )
    param_vision = ParameterFile(param_vision_path, allow_substs=True)

    node_vision = Node(
        namespace=namespace,
        package="opossum_vision",
        executable="vision_node.py",
        name="vision_node",
        parameters=[param_vision],
    )

    ld.add_action(node_ihm)
    ld.add_action(param_server)
    ld.add_action(node_action_sequencer)
    ld.add_action(node_com)
    ld.add_action(node_tf_broadcaster)
    ld.add_action(node_rplidar)
    ld.add_action(node_beacon_detector)
    ld.add_action(node_obstacle_extractor)
    ld.add_action(node_avoid_obstacle)
    ld.add_action(node_vision)

    return ld