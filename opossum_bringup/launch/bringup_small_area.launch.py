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
        parameters=[{
            "board_config": "small_objects",
            "year": 2026,
            "boundaries": [0.0, 1.0, 0.0, 2.0],
        }],
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
        namespace=namespace,
        executable="tf_broadcaster.py",
        name="tf_broadcaster_node",
        parameters=[param_loc, {"robot_names": ["main_robot"]}],
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

    node_vision_one = Node(
        namespace=namespace,
        package="opossum_vision",
        executable="vision_node.py",
        name="vision_node_one",  # ATTENTION: Doit matcher le YAML !
        parameters=[param_vision],
    )

    node_vision_two = Node(
        namespace=namespace,
        package="opossum_vision",
        executable="vision_node.py",
        name="vision_node_two", # ATTENTION: Doit matcher le YAML !
        parameters=[param_vision],
    )

    node_vision_three = Node(
        namespace=namespace,
        package="opossum_vision",
        executable="vision_node.py",
        name="vision_node_three",  # ATTENTION: Doit matcher le YAML !
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
    ld.add_action(node_vision_one)
    ld.add_action(node_vision_two)
    ld.add_action(node_vision_three)    

    return ld