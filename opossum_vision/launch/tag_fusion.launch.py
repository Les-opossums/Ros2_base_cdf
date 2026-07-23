"""Lance le noeud de fusion des tags ArUco (aruco_world / aruco_world_fused)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    robot_names = [n.strip() for n in
                   LaunchConfiguration("robot_names").perform(context).split(",")
                   if n.strip()]
    # Source unique des reglages (dont la latence camera calibree).
    param_file = PathJoinSubstitution(
        [FindPackageShare("opossum_vision"), "config", "tag_fusion_params.yaml"]
    )
    nodes = []
    for robot in robot_names:
        nodes.append(Node(
            package="opossum_vision",
            executable="tag_fusion_node.py",
            name="tag_fusion_node",
            namespace=robot,
            output="screen",
            parameters=[param_file],
        ))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot_names", default_value="main_robot"),
        OpaqueFunction(function=launch_setup),
    ])
