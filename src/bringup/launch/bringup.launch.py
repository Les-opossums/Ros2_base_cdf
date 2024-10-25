from launch import LaunchDescription
from launch import LaunchIntrospector  # noqa: E402
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    namespace = "robot_test"

    node_com = Node(
        package="uart_com",
        namespace=namespace,
        executable="uart_com_node",
        name="uart_com"
    )
    node_action_sequencer = Node(
        package="action_sequencer",
        namespace=namespace,
        executable="action_sequencer_node",
        name="action_sequencer"
    )
    node_big_brother = Node(
        package="big_brother",
        namespace=namespace,
        executable="big_brother_node",
        name="big_brother"
    )
    node_ihm = Node(
        package="ihm_ros",
        namespace=namespace,
        executable="ihm_ros_node",
        name="ihm_ros"
    )

    node_rplidar_top = Node(
        package="rplidar_ros",
        namespace="%s/top_lidar" % namespace,
        executable="rplidar_node",
        name="rp_lidar_top",
        parameters=[
            {"serial_port": "/dev/LidarHaut",
             "serial_baudrate": 256000,
             "channel_type": "serial",
             "frame_id": "laser",
             "inverted": False,
             "angle_compensate": True,
             "scan_mode": "Sensitivity"}
        ]
    )

    node_rplidar_bot = Node(
        package="rplidar_ros",
        namespace="%s/bottom_lidar" % namespace,
        executable="rplidar_node",
        name="rp_lidar_bot",
        parameters=[
            {"serial_port": "/dev/LidarBas",
             "serial_baudrate": 256000,
             "channel_type": "serial",
             "frame_id": "laser",
             "inverted": False,
             "angle_compensate": True,
             "scan_mode": "Sensitivity"}
        ]
    )

    ld.add_action(node_action_sequencer)

    ld.add_action(node_com)
    ld.add_action(node_rplidar_top)
    ld.add_action(node_rplidar_bot)
    ld.add_action(node_big_brother)
    ld.add_action(node_ihm)

    print(LaunchIntrospector().format_launch_description(ld))
    return ld
