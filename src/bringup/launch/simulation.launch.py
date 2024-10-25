from launch import LaunchDescription
from launch import LaunchIntrospector  # noqa: E402
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

import os

UTILS_PATH = os.environ['COLCON_PREFIX_PATH'][:-7] + "utils/"


def generate_launch_description():
    ld = LaunchDescription()

    launch_simu_robot = ExecuteProcess(
        cmd=['{}Robot_Simulator/Simu_Carte_User'.format(UTILS_PATH)],
        cwd='{}Robot_Simulator'.format(UTILS_PATH)
    )

    launch_pigpiod_em = ExecuteProcess(
        cmd=['python3',
             '{}Pigpiod_Emulator/tools/pigpiod_pepino_interface.py'
             .format(UTILS_PATH)],
    )

    launch_rqt = Node(
        package="rqt_gui",
        executable="rqt_gui",
        name="rqt_gui"
    )
    node_lidar_em = Node(
        package="lidar_emulator",
        namespace="robot_test",
        executable="LidarEmulNode",
        name="lidar_emulator",
    )

    node_sender = Node(
        package="uart_sender",
        namespace="robot_test",
        executable="uart_sender_node",
        name="uart_sender"
    )
    node_receiver = Node(
        package="uart_receiver",
        namespace="robot_test",
        executable="uart_receiver_node",
        name="uart_receiver"
    )
    node_action_sequencer = Node(
        package="action_sequencer",
        namespace="robot_test",
        executable="action_sequencer_node",
        name="action_sequencer"
    )

    ld.add_action(launch_simu_robot)
    ld.add_action(launch_rqt)

    ld.add_action(node_action_sequencer)

    ld.add_action(TimerAction(
        period=2.0,
        actions=[launch_pigpiod_em]
    )
    )
    ld.add_action(TimerAction(
        period=4.0,
        actions=[node_receiver, node_sender, node_lidar_em]
    )
    )
    print(LaunchIntrospector().format_launch_description(ld))
    return ld
