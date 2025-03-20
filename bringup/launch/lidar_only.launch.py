import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    namespace = "main_robot"

    config = os.path.join(
        get_package_share_directory("bringup"),
        "config",
        "lidar_params.yaml",
    )

    node_rplidar_top = Node(
        package="rplidar_ros",
        namespace="%s/top_lidar" % namespace,
        executable="rplidar_node",
        name="rp_lidar_top",
        parameters=[
            {
                "serial_port": "/dev/USBtty0",
                "serial_baudrate": 256000,
                "channel_type": "serial",
                "frame_id": "laser",
                "inverted": False,
                "angle_compensate": True,
                "scan_mode": "Sensitivity",
            }
        ],
    )

    ld.add_action(node_rplidar_top)

    return ld
