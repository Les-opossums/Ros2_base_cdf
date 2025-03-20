import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch import LaunchIntrospector  # noqa: E402
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    ld = LaunchDescription()

    lidar_only = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("bringup"), "launch"),
                "/lidar_only.launch.py",
            ]
        )
    )

    obj_detection = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("bringup"), "launch"),
                "/obj_detection.launch.py",
            ]
        )
    )

    robot_loc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("bringup"), "launch"),
                "/robot_loc.launch.py",
            ]
        )
    )

    comm = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("bringup"), "launch"),
                "/comm.launch.py",
            ]
        )
    )

    scripting = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("bringup"), "launch"),
                "/scripting.launch.py",
            ]
        )
    )

    ld.add_action(lidar_only)
    ld.add_action(obj_detection)
    ld.add_action(robot_loc)
    ld.add_action(comm)
    ld.add_action(scripting)

    print(LaunchIntrospector().format_launch_description(ld))
    return ld
