import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch import LaunchIntrospector  # noqa: E402
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    namespace = "main_robot"

    config = os.path.join(
        get_package_share_directory("bringup"),
        "config",
        "lidar_params.yaml",
    )

    node_obj_detector_haut = Node(
        package="obj_detector",
        namespace=namespace,
        executable="node_cluster",
        name="top_object_detector_node",
        parameters=[config],
    )

    node_beacon_detector = Node(
        package="beacon_detector",
        namespace=namespace,
        executable="beacon_detector_node",
        name="beacon_detector_node",
        parameters=[config],
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

    ld.add_action(node_obj_detector_haut)
    ld.add_action(node_beacon_detector)
    ld.add_action(node_rplidar_top)

    print(LaunchIntrospector().format_launch_description(ld))
    return ld
