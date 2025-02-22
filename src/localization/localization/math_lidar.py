# Import des librairies
from typing import Tuple, Union
import numpy as np
from .objet import RobotDatas

# Import des messages
import copy
from geometry_msgs.msg import Point

###########################################################################
############                  Elements de base                #############
###########################################################################


def dt2(
    PointA: Union[Point, np.ndarray], PointB: Union[Point, np.ndarray] = None
) -> float:
    """
    Paramètres:
        - PointA:   Point,  premier point
        - PointB:   Point,  second point

    Retourne:
        - la distance au carré entre le Point A et le Point B,
        ou le module au carré du Point A si un seul argument
    """
    if isinstance(PointA, Point):
        PointA = np.array([PointA.x, PointA.y])
    if isinstance(PointB, Point):
        PointB = np.array([PointB.x, PointB.y])
    if PointB is not None:
        return float(np.dot((PointA[:2] - PointB[:2]), (PointA[:2] - PointB[:2])))
    return float(np.dot(PointA[:2], PointA[:2]))


def dt(PointA: Point, PointB: Point = None) -> float:
    """
    Paramètres:
        - PointA:   Point,  premier point
        - PointB:   Point,  second point

    Retourne:
        - la distance entre le Point A et le Point B, ou le module du Point A si un seul
          argument
    """
    return np.sqrt(dt2(PointA, PointB))


def approx(distanceA: float, distanceB: float = 0, precision: float = 0.1) -> bool:
    """
    Paramètres:
        - distanceA:   float,  première distance
        - distanceB:   float,  seconde distance

    Retourne:
        - un booléen qui vérifie si les distances sont inférieures à une certaine précis
        ion
    """
    return abs(distanceA - distanceB) < precision

def cartesian_to_polar(x, y):
    rho = np.sqrt(x**2 + y**2)
    phi = np.arctan2(y, x)
    return rho, phi


def angle_in_range(alpha, lower, upper):
    if lower > upper:
        raise ValueError("Lower bound must be less than or equal to upper bound")
    alpha = np.mod(alpha, 2 * np.pi)
    lower = np.mod(lower, 2 * np.pi)
    upper = np.mod(upper, 2 * np.pi)
    if lower < upper:
        return lower <= alpha <= upper
    else:
        return lower <= alpha or alpha <= upper


###########################################################################
############               Changements de Bases              ##############
###########################################################################


def chgt_base_plateau_to_robot(point_to_transform: Point, robot_frame: Point) -> Point:
    cos_angle = np.cos(robot_frame.z)
    sin_angle = np.sin(robot_frame.z)
    OtoR = np.array(
        [
            [cos_angle, -sin_angle, robot_frame.x],
            [sin_angle, cos_angle, robot_frame.y],
            [0, 0, 1],
        ]
    )
    transformed_point = np.linalg.inv(OtoR) @ np.array(
        [point_to_transform.x, point_to_transform.y, 1]
    )
    return Point(x=transformed_point[0], y=transformed_point[1])


def chgt_base_robot_to_plateau(point_to_transform: Point, robot_frame: Point) -> Point:
    cos_angle = np.cos(robot_frame.z)
    sin_angle = np.sin(robot_frame.z)
    OtoR = np.array(
        [
            [cos_angle, -sin_angle, robot_frame.x],
            [sin_angle, cos_angle, robot_frame.y],
            [0, 0, 1],
        ]
    )
    transformed_point = OtoR @ np.array([point_to_transform.x, point_to_transform.y, 1])
    return Point(x=transformed_point[0], y=transformed_point[1])


###########################################################################
############                  Calcul des erreurs              #############
###########################################################################


def _calcul_erreur(data: RobotDatas, position: Point, balises: list[Point]):
    sum = 0
    if data.balise_A is not None:
        sum += dt2(chgt_base_robot_to_plateau(data.balise_A, position), balises[0])
    if data.balise_B is not None:
        sum += dt2(chgt_base_robot_to_plateau(data.balise_B, position), balises[1])
    if data.balise_C is not None:
        sum += dt2(chgt_base_robot_to_plateau(data.balise_C, position), balises[2])
    if data.balise_D is not None:
        sum += dt2(chgt_base_robot_to_plateau(data.balise_D, position), balises[3])
    return sum


###########################################################################
############                  Calcul des points              ##############
###########################################################################


def _calcul_point_3_balises(
    Balise1: Point,
    Balise2: Point,
    Balise3: Point,
    BaliseA: Point,
    BaliseB: Point,
    BaliseC: Point,
) -> Tuple[float, float]:
    """
    Paramètres:
        - Balise1:  RobotDatas, position de la Balise A dans le repère du robot
        - Balise2:  RobotDatas, position de la Balise B dans le repère du robot
        - Balise3:  RobotDatas, position de la Balise C dans le repère du robot
        - BaliseA:  RobotDatas, position de la Balise A dans le connu du plateau
        - BaliseB:  RobotDatas, position de la Balise B dans le connu du plateau
        - BaliseC:  RobotDatas, position de la Balise C dans le connu du plateau

    Effectue:
        - Le calcul à partir de ces paramètres, de la position du robot de la manière
        suivante:
        On pose:
            - distanceX = BaliseX.x ** 2 + BaliseX.y ** 2 (disance au carré de la balise
              X par rapport au robot)
            - x, y: position respective en x et y du robot
            - bX.x, bX.y, les position en x et y de la balise X
        On a alors:
            |   distance1 = (x - bA.x) ** 2 + (y - bA.y) ** 2
            |   distance2 = (x - bB.x) ** 2 + (y - bB.y) ** 2
            |   distance3 = (x - bC.x) ** 2 + (y - bC.y) ** 2
                            <=>
            |   distance1 = (x - bA.x) ** 2 + (y - bA.y) ** 2
            |   distance2 - distance1 = 2x * (bA.x - bB.x) + 2y * (bA.y - bB.y)
            + distanceB - distanceA
            |   distance3 - distance1 = 2x * (bA.x - bC.x) + 2y * (bA.y - bC.y)
            + distanceC - distanceA
                                        <=>
            |   distance1 = (x - bA.x) ** 2 + (y - bA.y) ** 2
            |   x = (distance2 - distanceB - (distance1 - distanceA) + 2y *
            (bB.y - bA.y)) / (2 * (bA.x - bB.x))
            |   distance3 - distance1 = 2x * (bA.x - bC.x) + 2y * (bA.y - bC.y)
              + distanceC - distanceA
                                        <=>
        Pour plus de simplicité dans la lecture, on pose:
            - deltaA = distance1 - distanceA
            - deltaB = distance2 - distanceB
            - deltaC = distance3 - distanceC
            - coeff = (bB.y - bA.y)/(bB.x - bA.x)
            - const = (deltaB - deltaA) / (2 * (bA.x - bB.x))

            |   distance1 = (x - bA.x) ** 2 + (y - bA.y) ** 2
            |   x = const + y * coeff
            |   distance3 - distance1 = 2 * (const * (bA.x - bC.x) + y * coeff *
            (bA.x - bC.x)) + 2y * (bA.y - bC.y) + distanceC - distanceA
                                        <=>
            |   distance1 = (x - bA.x) ** 2 + (y - bA.y) ** 2
            |   x = const + y * coeff
            |   y = (deltaC - deltaA - 2 * const * (bA.x - bC.x)) /
            (coeff * (bA.x - bC.x) + 2 * (bA.y - bC.y))

    Retourne:
        - deux floats, correspondant à la position x, y
    """

    distance1 = dt2(Balise1)
    distance2 = dt2(Balise2)
    distance3 = dt2(Balise3)
    distanceA = dt2(BaliseA)
    distanceB = dt2(BaliseB)
    distanceC = dt2(BaliseC)

    deltaA = distance1 - distanceA
    deltaB = distance2 - distanceB
    deltaC = distance3 - distanceC
    if abs(BaliseA.x - BaliseB.x) > 0.1:
        coeff = (BaliseB.y - BaliseA.y) / (BaliseA.x - BaliseB.x)
        const = (deltaB - deltaA) / (2 * (BaliseA.x - BaliseB.x))

        y = (deltaC - deltaA - 2 * const * (BaliseA.x - BaliseC.x)) / (
            2 * (coeff * (BaliseA.x - BaliseC.x) + (BaliseA.y - BaliseC.y))
        )
    elif abs(BaliseA.x - BaliseC.x) > 0.1:
        coeff = (BaliseC.y - BaliseA.y) / (BaliseA.x - BaliseC.x)
        const = (deltaC - deltaA) / (2 * (BaliseA.x - BaliseC.x))

        y = (deltaB - deltaA - 2 * const * (BaliseA.x - BaliseB.x)) / (
            2 * (coeff * (BaliseA.x - BaliseB.x) + (BaliseA.y - BaliseB.y))
        )
    x = const + y * coeff
    return x, y

def _calcul_point_2_balises(
    Balise1: Point, Balise2: Point, BaliseA: Point, BaliseB: Point
) -> Tuple[float, float, float, float]:
    """
    Paramètres:
        - Balise1:  Point, position de la Balise 1 dans le repère du robot
        - Balise2:  Point, position de la Balise 2 dans le repère du robot
        - BaliseA:  Point, position de la Balise A dans le connu du plateau
        - BaliseB:  Point, position de la Balise B dans le connu du plateau

    Effectue: Le calcul à partir de ces paramètres, de 2 positions potentielles du robot

    On pose:
            - distanceX = BaliseX.x² + BaliseX.y² (disance au carré de la balise
            X par rapport au robot)
            - x, y: position respective en x et y du robot
            - bX.x, bX.y, les position en x et y de la balise X
    On a alors:
        |   distance1 = (x - BaliseA.x)² + (y - BaliseA.y)²
        |   distance2 = (x - bB.x)² + (y - bB.y)²
                        <=>
        |   distance1 = (x - BaliseA.x)² + (y - BaliseA.y)²
        |   distance2 - distance1 = 2x * (BaliseA.x - bB.x) + 2y * (BaliseA.y - bB.y)
        + distanceB - distanceA
                        <=>
        |   distance1 = (x - BaliseA.x)² + (y - BaliseA.y)²
        |   x = (distance2 - distanceB - (distance1 - distanceA) + 2y *
        (bB.y - BaliseA.y)) / (2 * (BaliseA.x - bB.x))
                        <=>
    Pour plus de simplicité dans la lecture, on pose:
        - deltaA = distance1 - distanceA
        - deltaB = distance2 - distanceB
        - deltaC = distance3 - distanceC
        - coeff = (bB.y - BaliseA.y)/(bB.x - BaliseA.x)
        - const = (deltaB - deltaA) / (2 * (BaliseA.x - bB.x))

        |   x = const + coeff * y
        |   distance1 = (const + coeff * y - BaliseA.x)² + (y - BaliseA.y)²
                        <=>
        |   x = const + coeff * y
        |   (coeff² + 1) * y² + 2 * (coeff * (const - BaliseA.x) - BaliseA.y) * y +
          (const - BaliseA.x)² + BaliseA.y² - distance1 = 0

    Pour plus de simplicité à nouveau dans la lecture, on pose:
        - A = 1 + coeff²
        - B = 2 * (coeff * (const - BaliseA.x) - BaliseA.y)
        - C = (const - BaliseA.x)² + BaliseA.y² - distance1
    On a donc une équation du second degré, on s'attend donc à avoir 2 solutions.
                        <=>
        |   x = const + coeff * y
        |   A * y² + B * y + C = 0

    On pose:
        - delta = B² - 4 * A * C
            > si delta > 0:
                        <=>
        |   y1 = - (np.sqrt(delta) + B) / 2 * A     ou      y2 = (np.sqrt(delta) - B) /
          2 * A
        |   x1 = const + coeff * y1                 ou      x2 = const + coeff * y2

            > si delta = 0:
        |   y = - B / 2 * A
        |   x = const + coeff * y

            > si delta < 0:
        |   y = - 1
        |   x = - 1
    Retourne:
        x1, y1, x2, y2 les positions déterminées du robot
    """
    distance1 = dt2(Balise1)
    distance2 = dt2(Balise2)
    distanceA = dt2(BaliseA)
    distanceB = dt2(BaliseB)

    deltaA = distance1 - distanceA
    deltaB = distance2 - distanceB
    coeff = (BaliseB.y - BaliseA.y) / (BaliseA.x - BaliseB.x)
    const = (deltaB - deltaA) / (2 * (BaliseA.x - BaliseB.x))
    A = 1 + coeff**2
    B = 2 * (coeff * (const - BaliseA.x) - BaliseA.y)
    C = (const - BaliseA.x) ** 2 + BaliseA.y**2 - distance1
    delta = B**2 - 4 * A * C
    if delta > 0:
        y1, y2 = -(np.sqrt(delta) + B) / 2 * A, (np.sqrt(delta) - B) / 2 * A
        x1, x2 = const + coeff * y1, const + coeff * y2
        if x1 < -0.1 or x1 > 3.1 or y1 < -2.1 or y1 > 0.1:
            x1 = -1
            y1 = -1
        if x2 < -0.1 or x2 > 3.1 or y2 < -2.1 or y2 > 0.1:
            x2 = -1
            y2 = -1
        return x1, y1, x2, y2
    elif delta == 0:
        y = -B / 2 * A
        x = const + coeff * y
        if x < -0.1 or x > 3.1 or y < -2.1 or y > 0.1:
            x = -1
            y = -1
        return x, y, x, y
    else:
        return -1, -1, -1, -1


###########################################################################
############                  Calcul des angles               #############
###########################################################################
def _calcul_angle_31(
    RobotDatas: Point, BaliseRobot: Point, BalisePlateau: Point
) -> float:
    v2, v1 = Point(), Point()
    v1.x = BalisePlateau.x - RobotDatas.x
    v1.y = BalisePlateau.y - RobotDatas.y
    v2.x = BaliseRobot.x
    v2.y = BaliseRobot.y
    norm_v1, norm_v2 = dt(v1), dt(v2)
    cos_t = (v1.x * v2.x + v1.y * v2.y) / (norm_v1 * norm_v2)
    sin_t = (v2.x * v1.y - v2.y * v1.x) / (norm_v1 * norm_v2)
    angle_1 = np.arccos(cos_t)
    if v2.x * v1.y - v1.x * v2.y < 0:
        angle_1 = -angle_1
    angle_1 = np.mod(angle_1, 2 * np.pi)
    angle_2 = np.arcsin(sin_t)
    if v1.x * v2.x + v1.y * v2.y < 0:
        angle_2 = np.pi - angle_2
    angle_2 = np.mod(angle_2, 2 * np.pi)
    asin_t = abs(sin_t)
    acos_t = abs(cos_t)
    if asin_t < acos_t:
        return angle_2, angle_1, asin_t
    return angle_1, angle_2, acos_t

###########################################################################
############          Calcul des angles et points              ############
###########################################################################


def _res_all_4(
    Balise1, Balise2, Balise3, Balise4, BaliseA, BaliseB, BaliseC, BaliseD, choix
):
    diag = np.eye(8)
    B = np.transpose(
        np.array(
            [
                [
                    BaliseA.x,
                    BaliseA.y,
                    BaliseB.x,
                    BaliseB.y,
                    BaliseC.x,
                    BaliseC.y,
                    BaliseD.x,
                    BaliseD.y,
                ]
            ]
        )
    )
    A_uncoeff = np.array(
        [
            [1, 0, Balise1.x, -Balise1.y],
            [0, 1, Balise1.y, Balise1.x],
            [1, 0, Balise2.x, -Balise2.y],
            [0, 1, Balise2.y, Balise2.x],
            [1, 0, Balise3.x, -Balise3.y],
            [0, 1, Balise3.y, Balise3.x],
            [1, 0, Balise4.x, -Balise4.y],
            [0, 1, Balise4.y, Balise4.x],
        ]
    )
    A = np.dot(diag, A_uncoeff)
    try:
        X = np.linalg.solve(np.dot(np.transpose(A), A), np.dot(np.transpose(A), B))
        if abs(X[2]) < 1:
            if X[3] > 0:
                theta1 = np.arccos(X[2])
            else:
                theta1 = -np.arccos(X[2])
        elif abs(X[3] < 1):
            if X[2] < 0:
                theta1 = np.pi - np.arcsin(X[3])
            else:
                theta1 = np.arcsin(X[3])
        else:
            if X[2] < -1:
                theta1 = np.pi - np.arctan(X[3] / X[2])
            else:
                theta1 = np.arctan(X[3] / X[2])
        theta1 = np.mod(theta1, 2 * np.pi)
        return float(X[0]), float(X[1]), float(theta1)
    except:  # noqa: E722
        return 0, 0, 0


def _res_all_3(Balise1, Balise2, Balise3, BaliseA, BaliseB, BaliseC, choix):
    B = np.transpose(
        np.array([[BaliseA.x, BaliseA.y, BaliseB.x, BaliseB.y, BaliseC.x, BaliseC.y]])
    )
    A = np.array(
        [
            [1, 0, Balise1.x, -Balise1.y],
            [0, 1, Balise1.y, Balise1.x],
            [1, 0, Balise2.x, -Balise2.y],
            [0, 1, Balise2.y, Balise2.x],
            [1, 0, Balise3.x, -Balise3.y],
            [0, 1, Balise3.y, Balise3.x],
        ]
    )
    try:
        X = np.linalg.solve(np.dot(np.transpose(A), A), np.dot(np.transpose(A), B))
        if abs(X[2]) < 1:
            if X[3] > 0:
                theta1 = np.arccos(X[2])
            else:
                theta1 = -np.arccos(X[2])
        elif abs(X[3] < 1):
            if X[2] < 0:
                theta1 = np.pi - np.arcsin(X[3])
            else:
                theta1 = np.arcsin(X[3])
        else:
            if X[2] < -1:
                theta1 = np.pi - np.arctan(X[3] / X[2])
            else:
                theta1 = np.arctan(X[3] / X[2])
        theta1 = np.mod(theta1, 2 * np.pi)
        return float(X[0]), float(X[1]), float(theta1)
    except:  # noqa: E722
        return 0, 0, 0


def _res_all_2(Balise1, Balise2, BaliseA, BaliseB, choix):
    B = np.transpose(np.array([[BaliseA.x, BaliseA.y, BaliseB.x, BaliseB.y]]))
    A = np.array(
        [
            [1, 0, Balise1.x, -Balise1.y],
            [0, 1, Balise1.y, Balise1.x],
            [1, 0, Balise2.x, -Balise2.y],
            [0, 1, Balise2.y, Balise2.x],
        ]
    )
    try:
        X = np.linalg.solve(np.dot(np.transpose(A), A), np.dot(np.transpose(A), B))
        if abs(X[2]) < 1:
            if X[3] > 0:
                theta1 = np.arccos(X[2])
            else:
                theta1 = -np.arccos(X[2])
        elif abs(X[3] < 1):
            if X[2] < 0:
                theta1 = np.pi - np.arcsin(X[3])
            else:
                theta1 = np.arcsin(X[3])
        else:
            if X[2] < -1:
                theta1 = np.pi - np.arctan(X[3] / X[2])
            else:
                theta1 = np.arctan(X[3] / X[2])
        theta1 = np.mod(theta1, 2 * np.pi)
        return float(X[0]), float(X[1]), float(theta1)
    except:  # noqa: E722
        return 0, 0, 0

###########################################################################
############         Fonction de recherche des points        ##############
###########################################################################


def _trouver_positions_4_balises_1(
    balise, fixed_balises: list[Point, Point, Point, Point]
) -> None:
    return _calcul_point_3_balises(
        balise.balise_A,
        balise.balise_B,
        balise.balise_C,
        fixed_balises[0],
        fixed_balises[1],
        fixed_balises[2],
    )


def _trouver_positions_3_balises_1(
    balise, fixed_balises: list[Point, Point, Point, Point]
) -> None:
    if balise.balise_A is None:
        return _calcul_point_3_balises(
            balise.balise_B,
            balise.balise_C,
            balise.balise_D,
            fixed_balises[1],
            fixed_balises[2],
            fixed_balises[3],
        )
    elif balise.balise_B is None:
        return _calcul_point_3_balises(
            balise.balise_A,
            balise.balise_C,
            balise.balise_D,
            fixed_balises[0],
            fixed_balises[2],
            fixed_balises[3],
        )
    elif balise.balise_C is None:
        return _calcul_point_3_balises(
            balise.balise_A,
            balise.balise_B,
            balise.balise_D,
            fixed_balises[0],
            fixed_balises[1],
            fixed_balises[3],
        )
    elif balise.balise_D is None:
        return _calcul_point_3_balises(
            balise.balise_A,
            balise.balise_B,
            balise.balise_C,
            fixed_balises[0],
            fixed_balises[1],
            fixed_balises[2],
        )
###########################################################################
############         Fonction de recherche des angles        ##############
###########################################################################


# Fonction marchant avec 3eme méthode de calcul de l'angle sans vérif
def find_angle_31a(
    positionA: RobotDatas,
    x: float,
    y: float,
    fixed_balises: list[Point, Point, Point, Point],
):
    punto = Point()
    punto.x = x
    punto.y = y
    angle_tot = []
    angle_tot_bis = []
    min_lists = []
    if positionA.balise_A is not None:
        angle_temp_1, angle_temp_2, min_test = _calcul_angle_31(
            punto, positionA.balise_A, fixed_balises[0]
        )
        angle_tot.append(angle_temp_1)
        angle_tot_bis.append(angle_temp_1)
        angle_tot_bis.append(angle_temp_2)
        min_lists.append(min_test)
    if positionA.balise_B is not None:
        angle_temp_1, angle_temp_2, min_test = _calcul_angle_31(
            punto, positionA.balise_B, fixed_balises[1]
        )
        angle_tot.append(angle_temp_1)
        angle_tot_bis.append(angle_temp_1)
        angle_tot_bis.append(angle_temp_2)
        min_lists.append(min_test)
    if positionA.balise_C is not None:
        angle_temp_1, angle_temp_2, min_test = _calcul_angle_31(
            punto, positionA.balise_C, fixed_balises[2]
        )
        angle_tot.append(angle_temp_1)
        angle_tot_bis.append(angle_temp_1)
        angle_tot_bis.append(angle_temp_2)
        min_lists.append(min_test)
    if positionA.balise_D is not None:
        angle_temp_1, angle_temp_2, min_test = _calcul_angle_31(
            punto, positionA.balise_D, fixed_balises[3]
        )
        angle_tot.append(angle_temp_1)
        angle_tot_bis.append(angle_temp_1)
        angle_tot_bis.append(angle_temp_2)
        min_lists.append(min_test)
    
    min_test = 2
    for i in range(len(min_lists)):
        if min_lists[i] < min_test:
            index = i
            min_test = min_lists[i]
    return angle_tot[index]

###########################################################################
######        Fonction de recherche des angles et des points        #######
###########################################################################
def _trouver_positions_4_balises_3(
    balise, fixed_balises: list[Point, Point, Point, Point], choix
) -> None:
    return _res_all_4(
        balise.balise_A,
        balise.balise_B,
        balise.balise_C,
        balise.balise_D,
        fixed_balises[0],
        fixed_balises[1],
        fixed_balises[2],
        fixed_balises[3],
        choix,
    )


def _trouver_positions_3_balises_3(balise, fixed_balises, choix) -> None:
    if balise.balise_A is None:
        return _res_all_3(
            balise.balise_B,
            balise.balise_C,
            balise.balise_D,
            fixed_balises[1],
            fixed_balises[2],
            fixed_balises[3],
            choix,
        )
    elif balise.balise_B is None:
        return _res_all_3(
            balise.balise_A,
            balise.balise_C,
            balise.balise_D,
            fixed_balises[0],
            fixed_balises[2],
            fixed_balises[3],
            choix,
        )
    elif balise.balise_C is None:
        return _res_all_3(
            balise.balise_A,
            balise.balise_B,
            balise.balise_D,
            fixed_balises[0],
            fixed_balises[1],
            fixed_balises[3],
            choix,
        )
    elif balise.balise_D is None:
        return _res_all_3(
            balise.balise_A,
            balise.balise_B,
            balise.balise_C,
            fixed_balises[0],
            fixed_balises[1],
            fixed_balises[2],
            choix,
        )


def _trouver_positions_2_balises_3(balise, fixed_balises, choix) -> None:
    if balise.balise_A is None and balise.balise_B is None:
        return _res_all_2(
            balise.balise_C,
            balise.balise_D,
            fixed_balises[2],
            fixed_balises[3],
            choix,
        )
    elif balise.balise_A is None and balise.balise_C is None:
        return _res_all_2(
            balise.balise_B,
            balise.balise_D,
            fixed_balises[1],
            fixed_balises[3],
            choix,
        )
    elif balise.balise_A is None and balise.balise_D is None:
        return _res_all_2(
            balise.balise_B,
            balise.balise_C,
            fixed_balises[1],
            fixed_balises[2],
            choix,
        )
    elif balise.balise_B is None and balise.balise_C is None:
        return _res_all_2(
            balise.balise_A,
            balise.balise_D,
            fixed_balises[0],
            fixed_balises[3],
            choix,
        )
    elif balise.balise_B is None and balise.balise_D is None:
        return _res_all_2(
            balise.balise_A,
            balise.balise_C,
            fixed_balises[0],
            fixed_balises[2],
            choix,
        )
    elif balise.balise_C is None and balise.balise_D is None:
        return _res_all_2(
            balise.balise_A,
            balise.balise_B,
            fixed_balises[0],
            fixed_balises[1],
            choix,
        )
###########################################################################
############          Fonctions completes de recherche          ###########
###########################################################################
def _calcul_position_complete_11a(
    data: RobotDatas, fixed_balises, nombre: int
) -> float:
    if nombre == 4:
        x, y = _trouver_positions_4_balises_1(data, fixed_balises)
    elif nombre == 3:
        x, y = _trouver_positions_3_balises_1(data, fixed_balises)
    return x, y, find_angle_31a(data, x, y, fixed_balises)

def _calcul_positions_complete_3(
    datas: RobotDatas, fixed_balises, nombre: int, choix: int = 1
) -> None:
    if nombre == 4:
        return _trouver_positions_4_balises_3(datas, fixed_balises, choix)
    elif nombre == 3:
        return _trouver_positions_3_balises_3(datas, fixed_balises, choix)
    elif nombre == 2:
        return _trouver_positions_2_balises_3(datas, fixed_balises, choix)

###########################################################################
############            Merge des toutes les donnees            ###########
###########################################################################


def compute_potential_positions(
    datas: RobotDatas,
    fixed_beacons: list[Point],
    nombre: int,
    boundaries: list[float, float, float, float],
) -> None:
    for beacon in datas:
        err = []
        positions_all = []
        position_found = Point()
        position_found.x, position_found.y, position_found.z = (
            _calcul_positions_complete_3(beacon, fixed_beacons, nombre, 1)
        )
        dt_min = _calcul_erreur(beacon, position_found, fixed_beacons)
        err.append(dt_min)
        positions_all.append(copy.deepcopy(position_found))
        test = 1
        position = Point()
        position.x, position.y, position.z = (
            _calcul_positions_complete_3(beacon, fixed_beacons, nombre, 1)
        )
        position.z = find_angle_31a(beacon, position.x, position.y, fixed_beacons)
        dt = _calcul_erreur(beacon, position, fixed_beacons)
        if dt < dt_min:
                position_found.x, position_found.y, position_found.z = (
                    position.x,
                    position.y,
                    position.z,
                )
                dt_min = dt
                test = 2
        err.append(dt)
        positions_all.append(copy.deepcopy(position))

        if nombre > 2:
            position = Point()
            position.x, position.y, position.z = _calcul_position_complete_11a(
                beacon, fixed_beacons, nombre
            )
            dt = _calcul_erreur(beacon, position, fixed_beacons)
            err.append(dt)
            a = [position.x, position.y, position.z]
            positions_all.append(a)
            if dt < dt_min:
                position_found.x, position_found.y, position_found.z = (
                    position.x,
                    position.y,
                    position.z,
                )
                dt_min = dt
                test = 3

        beacon.position.x = position_found.x
        beacon.position.y = position_found.y
        beacon.position.z = position_found.z
        beacon.erreur = dt_min

    copy_beacons = datas.copy()

    for beacon in copy_beacons:
        if (
            beacon.position.x < boundaries[0] - 0.2  # Left
            or beacon.position.x > boundaries[1] + 0.2  # Right
            or beacon.position.y < boundaries[2] - 0.2  # Bottom
            or beacon.position.y > boundaries[3] + 0.2  # Top
        ):
            datas.remove(beacon)
    return datas, test, err, positions_all


def make_point(beacon: list[float, float]) -> Point:
    point = Point()
    point.x = beacon[0]
    point.y = beacon[1]
    return point

def get_sign_vect_product(beacon1, beacon2, beacon3):
    vec1 = [beacon2.x - beacon1.x, beacon2.y - beacon1.y]
    vec2 = [beacon3.x - beacon1.x, beacon3.y - beacon1.y]
    return np.sign(vec1[0] * vec2[1] - vec1[1] * vec2[0])
