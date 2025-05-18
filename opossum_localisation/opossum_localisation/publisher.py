"""Convert point from the world frame to robot one."""

from geometry_msgs.msg import Point
from opossum_msgs.msg import LidarLoc
import math


def publish_pose_from_lidar(robot_datas) -> None:
    """Publish datas get from lidar."""
    rdatas = LidarLoc()
    value_approx = 10000
    rdatas.robot_position.x = (
        int(value_approx * robot_datas["position"][0]) / value_approx
    )
    rdatas.robot_position.y = (
        int(value_approx * robot_datas["position"][1]) / value_approx
    )
    rdatas.robot_position.z = robot_datas["position"][2] % (2 * math.pi)
    rdatas.other_robot_position = []
    rdatas.balises = [
        Point(x=robot_datas["beacons"][0][0], y=robot_datas["beacons"][0][1]),
        Point(x=robot_datas["beacons"][1][0], y=robot_datas["beacons"][1][1]),
        Point(x=robot_datas["beacons"][2][0], y=robot_datas["beacons"][2][1]),
        Point(x=robot_datas["beacons"][3][0], y=robot_datas["beacons"][3][1]),
    ]
    for i in robot_datas["other_robots"]:
        tmp_stock = Point()
        tmp_stock.x = int(value_approx * i[0]) / value_approx
        tmp_stock.y = int(value_approx * i[1]) / value_approx
        rdatas.other_robot_position.append(tmp_stock)
    return rdatas
