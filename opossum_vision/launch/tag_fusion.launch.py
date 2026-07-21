"""Lance le noeud de fusion des tags ArUco (aruco_world / aruco_world_fused)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    robot_names = [n.strip() for n in
                   LaunchConfiguration("robot_names").perform(context).split(",")
                   if n.strip()]
    nodes = []
    for robot in robot_names:
        nodes.append(Node(
            package="opossum_vision",
            executable="tag_fusion_node.py",
            name="tag_fusion_node",
            namespace=robot,
            output="screen",
            parameters=[{
                "camera_extra_latency_s": 0.0,
                "fuse_alpha": 0.35,
                "gate_m": 0.12,
                "match_m": 0.15,
                "min_hits": 3,
            }],
        ))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot_names", default_value="main_robot"),
        OpaqueFunction(function=launch_setup),
    ])
