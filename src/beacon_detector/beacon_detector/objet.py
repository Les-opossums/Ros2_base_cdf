# Import des librairies
import numpy as np

# Import des messages
from geometry_msgs.msg import Point


class ChooseColor:
    def __init__(self, red, green, blue):
        self.red = red
        self.green = green
        self.blue = blue


class RobotDatas:
    """
    Objectif:
        - Permettre d'utiliser cette classe pour ordonner correctement les informations
        nécessaire pour le positionement avec le lidar, comme la position des balises,
        du robot, ou même son angkle d'orientation par rappport au plateau

    ---
    Paramètres:
        - x:            float,          position en x du robot par rapport
          au repère du plateau
        - y:            float,          position en y du robot par rapport
          au repère du plateau
        - balise_A:     Point,          position de la balise A par rapport
          au repère du robot
        - balise_B:     Point,          position de la balise B par rapport
          au repère du robot
        - balise_C:     Point,          position de la balise C par rapport
          au repère du robot
        - balise_D:     Point,          position de la balise D par rapport
          au repère du robot
        - angle:        float,          angle du repère du robot par rapport
          au repère du plateau
        - other_robots: list(Point),    liste des positions des autres robots
    """

    def __init__(
        self,
        balise_A: Point = None,
        balise_B: Point = None,
        balise_C: Point = None,
        balise_D: Point = None,
    ) -> None:
        self.balise_A, self.balise_B, self.balise_C, self.balise_D = (
            balise_A,
            balise_B,
            balise_C,
            balise_D,
        )
        self.position = Point()
        self.other_robots = []

def dt(PointA: Point, PointB: Point = None) -> float:
    if PointB is not None:
        return np.sqrt(
            (PointB.x - PointA.x) * (PointB.x - PointA.x)
            + (PointB.y - PointA.y) * (PointB.y - PointA.y)
        )
    return np.sqrt(PointA.x * PointA.x + PointA.y * PointA.y)
