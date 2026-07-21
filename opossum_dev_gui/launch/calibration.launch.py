"""Lance le gestionnaire de calibration camera (pilotable depuis l'IHM web)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    robot_names = [n.strip() for n in
                   LaunchConfiguration("robot_names").perform(context).split(",")
                   if n.strip()]
    return [Node(
        package="opossum_dev_gui",
        executable="calibration_manager.py",
        name="calibration_manager",
        namespace=robot,
        output="screen",
    ) for robot in robot_names]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot_names", default_value="main_robot"),
        OpaqueFunction(function=launch_setup),
    ])
