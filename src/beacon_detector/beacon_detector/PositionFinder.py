import numpy as np
from .math_lidar import *


class PositionFinder:

    def __init__(
        self, fixed_beacons, boundaries, precision=0.1, init_position=None
    ) -> None:
        self.previous_robot = None
        self.current_robot = None
        self.true_valor = False
        self.initialisation = False
        self.registre_init = []
        self.tour_repr = 0
        self.tour_reinit = 0
        self.tour_test = 0
        self.precision = precision
        self.BaliseA = fixed_beacons[0]
        self.BaliseB = fixed_beacons[1]
        self.BaliseC = fixed_beacons[2]
        self.BaliseD = fixed_beacons[3]
        self.boundaries = boundaries
        self.fixed_beacons = fixed_beacons
        if init_position is not None:
            self.previous_robot = RobotDatas()
            self.previous_robot.position = Point(
                x=init_position[0], y=init_position[1], z=np.pi * init_position[2] / 180
            )
            self.previous_robot.balise_A = chgt_base_plateau_to_robot(
                self.fixed_beacons[0], self.previous_robot.position
            )
            self.previous_robot.balise_B = chgt_base_plateau_to_robot(
                self.fixed_beacons[1], self.previous_robot.position
            )
            self.previous_robot.balise_C = chgt_base_plateau_to_robot(
                self.fixed_beacons[2], self.previous_robot.position
            )
            self.previous_robot.balise_D = chgt_base_plateau_to_robot(
                self.fixed_beacons[3], self.previous_robot.position
            )
            self.initialisation = True

    # 1
    def _update_data(self) -> None:
        self.tour_repr += 1
        if not self.true_valor and self.initialisation:
            self.tour_test += 1
        elif self.true_valor:
            self.tour_test = 0
        if self.tour_test >= 30:
            self.tour_test = 0
            self.initialisation = False
        if not self.initialisation:
            self.tour_reinit += 1

    # 3
    def _compare_previous_found_positions(self, potential_robot_datas) -> RobotDatas:
        robot_datas = potential_robot_datas.copy()
        dst_min = robot_datas[0].erreur
        best_match = robot_datas[0]
        for robot_data in robot_datas[1:]:
            dst_test = robot_data.erreur
            if dst_test < dst_min:
                dst_min = dst_test
                best_match = robot_data
        if dt(self.previous_robot.position, best_match.position) < self.precision:
            self.current_robot = best_match
            self.true_valor = True
        else:
            self.true_valor = False
            self.current_robot = self.previous_robot

    # 4
    def _recreate_balise(self) -> None:
        if self.current_robot.position.z is not None:
            if self.current_robot.balise_A is None:
                self.current_robot.balise_A = chgt_base_plateau_to_robot(
                    self.BaliseA, self.current_robot.position
                )
            if self.current_robot.balise_B is None:
                self.current_robot.balise_B = chgt_base_plateau_to_robot(
                    self.BaliseB, self.current_robot.position
                )
            if self.current_robot.balise_C is None:
                self.current_robot.balise_C = chgt_base_plateau_to_robot(
                    self.BaliseC, self.current_robot.position
                )
            if self.current_robot.balise_D is None:
                self.current_robot.balise_D = chgt_base_plateau_to_robot(
                    self.BaliseD, self.current_robot.position
                )

    # 5
    def _find_robots_on_plateau(self, obstacles) -> None:
        self.current_robot.other_robots = []
        cos_theta = np.cos(self.current_robot.position.z)
        sin_theta = np.sin(self.current_robot.position.z)
        for obstacle in obstacles:
            x, y = obstacle.x, obstacle.y
            x1, y1 = (cos_theta * x - sin_theta * y + self.current_robot.position.x), (
                cos_theta * y + sin_theta * x + self.current_robot.position.y
            )
            if (
                (x1 > self.boundaries[0])
                and (x1 < self.boundaries[1])
                and (y1 > self.boundaries[2])
                and (y1 < self.boundaries[3])
            ):
                if (
                    (
                        not approx(
                            dt(obstacle, self.current_robot.balise_A), precision=0.01
                        )
                    )
                    and (
                        not approx(
                            dt(obstacle, self.current_robot.balise_B), precision=0.01
                        )
                    )
                    and (
                        not approx(
                            dt(obstacle, self.current_robot.balise_C), precision=0.01
                        )
                    )
                    and (
                        not approx(
                            dt(obstacle, self.current_robot.balise_D), precision=0.01
                        )
                    )
                ):
                    positionement = np.array([x1, y1, 1])
                    self.current_robot.other_robots.append(positionement)

    # 2
    def _init_finding(self, potential_robot_datas) -> None:
        if len(self.registre_init) == 0:
            liste_tool_globale = []
            liste_balise = potential_robot_datas.copy()
            while len(liste_balise) > 0:
                ver = False
                bal = liste_balise.pop()
                for element in liste_tool_globale:
                    if not ver:
                        if (
                            abs(element[0].position.x - bal.position.x) < self.precision
                            and abs(element[0].position.y - bal.position.y)
                            < self.precision
                        ):
                            ver = True
                            element[1] += 1
                            element[0].position.x = (
                                element[0].position.x * (element[1] - 1)
                                + bal.position.x
                            ) / element[1]
                if not ver:
                    liste_tool_globale.append([bal, 1])

            self.registre_init.append(liste_tool_globale)
        else:
            liste_tool_globale = []
            liste_balise = potential_robot_datas.copy()
            liste_prev = self.registre_init[-1].copy()
            liste_rm = liste_prev.copy()
            while len(liste_balise) > 0:
                ver = False
                bal = liste_balise.pop()
                for element in liste_tool_globale:
                    if not ver:
                        if (
                            abs(element[0].position.x - bal.position.x) < self.precision
                            and abs(element[0].position.y - bal.position.y)
                            < self.precision
                        ):
                            ver = True
                            element[1] += 1
                            element[0].position.x = (
                                element[0].position.x * (element[1] - 1)
                                + bal.position.x
                            ) / element[1]
                if not ver:
                    for element in liste_prev:
                        if not ver:
                            if (
                                abs(element[0].position.x - bal.position.x)
                                < self.precision
                                and abs(element[0].position.y - bal.position.y)
                                < self.precision
                            ):
                                ver = True
                                liste_tool_globale.append([bal, 1 + element[1]])
                                liste_rm.remove(element)
                if not ver:
                    liste_tool_globale.append([bal, 1])
                liste_prev = liste_rm.copy()
            for element in liste_prev:
                liste_tool_globale.append(element)
            self.registre_init.append(liste_tool_globale)
        if self.tour_reinit >= 4:
            if len(self.registre_init) > 0:
                if len(self.registre_init[-1]) > 0:
                    donnees = self.registre_init[-1][0][0]
                    max = self.registre_init[-1][0][1]
                    for balise in self.registre_init[-1][1:]:
                        if balise[1] > max:
                            max = balise[1]
                            donnees = balise[0]
                    donnees.position.z = find_angle_31a(
                        donnees,
                        donnees.position.x,
                        donnees.position.y,
                        self.fixed_beacons,
                    )
                    self.current_robot = donnees
                    self._recreate_balise()
                    self.true_valor = True
                    self.initialisation = True
            self.tour_reinit = 0
            self.tour_test = 0
            self.registre_init = []

    # Used in beacon_detector_node.py
    def donnees_finales(
        self, nb_potential_beacons, potential_beacons, obstacles
    ) -> None:
        self._update_data()
        if nb_potential_beacons > 0:
            potential_robot_datas, _, _, _ = compute_potential_positions(
                potential_beacons,
                self.fixed_beacons,
                nb_potential_beacons,
                self.boundaries,
            )

            if len(potential_robot_datas) > 0:
                if not self.initialisation:
                    self._init_finding(potential_robot_datas)
                else:
                    self._compare_previous_found_positions(potential_robot_datas)
                if self.initialisation and self.true_valor:
                    self._recreate_balise()
                    self._find_robots_on_plateau(obstacles)
                    return self.current_robot
        self.true_valor = False
        return None
