from launch import LaunchDescription
from launch import LaunchIntrospector  # noqa: E402
from launch.actions import ExecuteProcess, TimerAction, \
                            SetEnvironmentVariable, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

import os


def generate_launch_description():
    UTILS_PATH = os.environ['COLCON_PREFIX_PATH'][:-7] + "utils/"

    # Get the path to the gazebo pkg
    pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')

    # Pose where we want to spawn the robot
    spawn_x_val = '1'
    spawn_y_val = '1'
    spawn_z_val = '0.2'
    spawn_yaw_val = '0.00'

    # Set the path to the world loaded at launch
    world = '{}gazebo_models/world.sdf'.format(UTILS_PATH)

    return LaunchDescription([
        # Add the models environment variable
        SetEnvironmentVariable(
            name='IGN_GAZEBO_RESOURCE_PATH',
            value='{}gazebo_models/'.format(UTILS_PATH)
        ),
        # Start Gazebo server
        #IncludeLaunchDescription(
        #    PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        #    launch_arguments={'world': world}.items()
        #),
        ExecuteProcess(
            cmd=['ign', 'gazebo', world],
            output='screen'
        ),
        ExecuteProcess(
            cmd=["ign", "service", "-s", "/world/world_demo/create",
                 "--reqtype", "ignition.msgs.EntityFactory",
                 "--reptype", "ignition.msgs.Boolean",
                 "--timeout", "5000",
                 "--req",
                 "sdf_filename: \"{}gazebo_models/robot_cdf_2024/urdf/robot_cdf_2024.urdf\", ".format(UTILS_PATH)
                 + "name: \"robot_cdf_2024\", "
                 + "pose: {{position: {{x: {}, y: {}, z: {}}}, orientation{{x: -0.7071068, y: 0, z: 0, w: 0.7071068}}}}".format(spawn_x_val, spawn_y_val, spawn_z_val)
                 ],
            output='screen'
        )
    ])
