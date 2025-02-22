# Import des librairies
import numpy as np
import time
from typing import *

# from tf.transformations import quaternion_from_euler
from .objet import ChooseColor, RobotDatas
from .math_lidar import chgt_base_robot_to_plateau

# Import des messages
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from cdf_msgs.msg import LidarLoc


def publicate_donnees_lidar(robot_datas, value: bool = True) -> None:
    """
    Paramètres:
        - donnees:      PositionFinder,     données qu'on a déterminées
        - emission:     LidarLoc,          données, qu'on crée pour publier

    Effectue:
        - donnees va affecter toutes ses données à emission
    """
    finish_position = LidarLoc()
    value_approx = 100
    finish_position.robot_position.x = (
        int(value_approx * robot_datas.position.x) / value_approx
    )
    finish_position.robot_position.y = (
        int(value_approx * robot_datas.position.y) / value_approx
    )
    finish_position.robot_position.z = robot_datas.position.z
    finish_position.other_robot_position = []
    finish_position.balises = [
        robot_datas.balise_A,
        robot_datas.balise_B,
        robot_datas.balise_C,
        robot_datas.balise_D,
    ]
    for i in robot_datas.other_robots:
        tmp_stock = Point()
        tmp_stock.x = int(value_approx * i[0]) / value_approx
        tmp_stock.y = int(value_approx * i[1]) / value_approx
        finish_position.other_robot_position.append(tmp_stock)
    return finish_position

def publicate_donnees_zc(list_robots) -> None:
    """
    Paramètres:
        - donnees:      PositionFinder,     données qu'on a déterminées
        - emission:     LidarLoc,          données, qu'on crée pour publier

    Effectue:
        - donnees va affecter toutes ses données à emission
    """
    finish_position = LidarLoc()
    for i in list_robots:
        tmp_stock = Point()
        tmp_stock.x = i[0]
        tmp_stock.y = i[1]
        finish_position.other_robot_position.append(tmp_stock)
    return finish_position


def display_lidar_position(robot_datas: RobotDatas, id=0) -> None:
    marker_array = MarkerArray()
    main_marker = Marker()
    color = ChooseColor(0.7, 0.1, 0.1)
    marker_pos_robot = create_marker(
        id_1=id,
        type_1=main_marker.CUBE,
        angle=robot_datas.position.z,
        scale_x=0.15,
        scale_y=0.35,
        pos_x=robot_datas.position.x,
        pos_y=robot_datas.position.y,
        color=color,
        scale_z=0.35,
        frame_id="laser",
    )
    marker_array.markers.append(marker_pos_robot)
    return marker_array


def display_other_robots_positions(other_robots) -> None:
    marker_array = MarkerArray()
    main_marker = Marker()
    for i, robot in enumerate(other_robots):
        color = ChooseColor(0.1, 0.1, 0.7)
        marker_pos_other = create_marker(
            id_1=100 + i,
            type_1=main_marker.CYLINDER,
            angle=0,
            scale_x=0.1,
            scale_y=0.1,
            pos_x=robot[0],
            pos_y=robot[1],
            color=color,
            scale_z=0.15,
            frame_id="laser",
        )
        marker_array.markers.append(marker_pos_other)
    return marker_array


def diplay_fixed_beacons_positions(fixed_beacons: list[Point]) -> None:
    marker_array = MarkerArray()
    main_marker = Marker()
    for i, bal in enumerate(fixed_beacons):
        color = ChooseColor(0.0, 1.0, 0.5)
        marker_bal = create_marker(
            id_1=20 + i,
            type_1=main_marker.CYLINDER,
            angle=0,
            scale_x=0.1,
            scale_y=0.1,
            pos_x=bal.x,
            pos_y=bal.y,
            color=color,
            scale_z=0.6,
            frame_id="laser",
        )
        marker_array.markers.append(marker_bal)
    return marker_array


def diplay_beacons_positions_found(robot_datas: RobotDatas):
    marker_array = MarkerArray()
    main_marker = Marker()
    liste = [
        chgt_base_robot_to_plateau(robot_datas.balise_A, robot_datas.position),
        chgt_base_robot_to_plateau(robot_datas.balise_B, robot_datas.position),
        chgt_base_robot_to_plateau(robot_datas.balise_C, robot_datas.position),
        chgt_base_robot_to_plateau(robot_datas.balise_D, robot_datas.position),
    ]
    for i, bal in enumerate(liste):
        color = ChooseColor(1, 0.2, 0.3)
        marker_bal = create_marker(
            id_1=15 + i,
            type_1=main_marker.CYLINDER,
            angle=0,
            scale_x=0.1,
            scale_y=0.1,
            pos_x=bal.x,
            pos_y=bal.y,
            color=color,
            scale_z=0.6,
            frame_id="laser",
        )
        marker_array.markers.append(marker_bal)
    return marker_array


def display_playground(boundaries: list[float]) -> None:
    # boundaries are displayed as follows: [x_min, x_max, y_min, y_max]
    marker_array = MarkerArray()
    center_pg_x = (boundaries[0] + boundaries[1]) / 2
    center_pg_y = (boundaries[2] + boundaries[3]) / 2
    main_marker = Marker()
    color = ChooseColor(0.4, 0.5, 0.4)
    marker_floor = create_marker(
        id_1=30,
        type_1=main_marker.CUBE,
        angle=0,
        scale_x=boundaries[1] - boundaries[0],
        scale_y=boundaries[3] - boundaries[2],
        pos_x=center_pg_x,
        pos_y=center_pg_y,
        color=color,
        scale_z=0.05,
        pos_z=-0.025,
        frame_id="laser",
    )
    marker_array.markers.append(marker_floor)
    return marker_array

def create_marker(
    id_1: int,
    type_1: int,
    angle: float,
    scale_x: float,
    scale_y: float,
    pos_x: float,
    pos_y: float,
    color: ChooseColor,
    scale_z: float = 0.05,
    sign: float = 0,
    transparency: float = 1,
    action: int = 0,
    pos_z=None,
    frame_id=None,
):
    ## Be careful due to the C binding probably, floats have to be floats, and strings have to be strings for frame id for example.
    marker = Marker()
    if frame_id is None:
        marker.header.frame_id = "base"
    else:
        marker.header.frame_id = frame_id
    marker.ns = "robot_position"
    marker.id = id_1
    marker.type = type_1
    marker.action = action
    marker.frame_locked = False
    marker.pose.position.x = float(pos_x)
    marker.pose.position.y = float(pos_y)
    if pos_z is None:
        marker.pose.position.z = float(scale_z / 2)
    else:
        marker.pose.position.z = float(pos_z)

    # Convert yaw angle to quaternion
    q = get_quaternion_from_euler(0, sign, angle + np.pi)
    marker.pose.orientation.x = float(q[0])
    marker.pose.orientation.y = float(q[1])
    marker.pose.orientation.z = float(q[2])
    marker.pose.orientation.w = float(q[3])

    marker.scale.x = float(scale_x)
    marker.scale.y = float(scale_y)
    marker.scale.z = float(scale_z)

    marker.color.a = float(transparency)
    marker.color.r = float(color.red)
    marker.color.g = float(color.green)
    marker.color.b = float(color.blue)

    return marker


# def remove_marker(id: int):
#     marker_to_remove_1, marker_to_remove_2 = Marker(), Marker()
#     marker_to_remove_1.id = 100 + id
#     marker_to_remove_2.id = 200 + id
#     marker_to_remove_1.action = marker_to_remove_1.DELETE
#     marker_to_remove_2.action = marker_to_remove_2.DELETE

#     return marker_to_remove_1, marker_to_remove_2


def transform_to_quaternion(point: Point):
    module = np.sqrt(point.x**2 + point.y**2)
    if module > 0.01:
        ang = np.arccos(point.x / module)
    else:
        ang = 0
    if point.y >= 0:
        sign = 0
    else:
        sign = np.pi
    return module, sign, ang


# def publish_debug(lidar_t: ListeT, pub: rospy.Publisher, debug_nb_bal: int, debug_nb_liste_bal: int, debug_positions_pot: list, rangement: RangementBalise):
#     data_to_send = DebugLidar()
#     data_to_send.distance_AB.data = dt(tool_glob.BaliseA, tool_glob.BaliseB)
#     data_to_send.distance_AC.data = dt(tool_glob.BaliseA, tool_glob.BaliseC)
#     data_to_send.distance_BC.data = dt(tool_glob.BaliseB, tool_glob.BaliseC)
#     data_to_send.distance_AD.data = dt(tool_glob.BaliseA, tool_glob.BaliseD)
#     data_to_send.distance_BD.data = dt(tool_glob.BaliseB, tool_glob.BaliseD)
#     data_to_send.distance_CD.data = dt(tool_glob.BaliseC, tool_glob.BaliseD)
#     if lidar_t is not None:
#         data_to_send.distance_AB_test.data = dt(lidar_t.balises[0], lidar_t.balises[1])
#         data_to_send.distance_AC_test.data = dt(lidar_t.balises[0], lidar_t.balises[2])
#         data_to_send.distance_BC_test.data = dt(lidar_t.balises[1], lidar_t.balises[2])
#         data_to_send.distance_AD_test.data = dt(lidar_t.balises[0], lidar_t.balises[3])
#         data_to_send.distance_BD_test.data = dt(lidar_t.balises[1], lidar_t.balises[3])
#         data_to_send.distance_CD_test.data = dt(lidar_t.balises[2], lidar_t.balises[3])
#     if rangement is not None:
#         data_to_send.pot_BA = rangement.liste_balise_A
#         data_to_send.pot_BB = rangement.liste_balise_B
#         data_to_send.pot_BC = rangement.liste_balise_C
#         data_to_send.pot_BD = rangement.liste_balise_D
#         data_to_send.nb_balises.data = debug_nb_bal
#         data_to_send.nb_liste_balises.data = debug_nb_liste_bal
#         data_to_send.positions_pot = debug_positions_pot
#     pub.publish(data_to_send)


def get_quaternion_from_euler(roll, pitch, yaw):
    """
    Convert an Euler angle to a quaternion.

    Input
      :param roll: The roll (rotation around x-axis) angle in radians.
      :param pitch: The pitch (rotation around y-axis) angle in radians.
      :param yaw: The yaw (rotation around z-axis) angle in radians.

    Output
      :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
    """
    qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(
        roll / 2
    ) * np.sin(pitch / 2) * np.sin(yaw / 2)
    qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(
        roll / 2
    ) * np.cos(pitch / 2) * np.sin(yaw / 2)
    qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(
        roll / 2
    ) * np.sin(pitch / 2) * np.cos(yaw / 2)
    qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(
        roll / 2
    ) * np.sin(pitch / 2) * np.sin(yaw / 2)

    return [qx, qy, qz, qw]
