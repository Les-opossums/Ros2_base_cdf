"""Compute the position of the robot using the beacons estmated with the lidar."""
import numpy as np
from .math_lidar import (
    convert_world_to_robot,
    dt,
    removearray,
    find_angle,
    find_position,
)


class PositionFinder:
    """Find the best estimation of the position."""

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
        self.boundaries = boundaries
        self.fixed_beacons = fixed_beacons
        if init_position is not None:
            self.previous_robot = {}
            self.previous_robot["position"] = np.array(
                [init_position[0], init_position[1], np.pi * init_position[2] / 180]
            )
            self.previous_robot["beacons"] = [
                convert_world_to_robot(beacon, self.previous_robot["position"])
                for beacon in fixed_beacons
            ]
            self.initialisation = True

    # 1
    def _update_data(self) -> None:
        """Update all data of the node before starting the computation."""
        self.tour_repr += 1
        if not self.true_valor and self.initialisation:
            self.tour_test += 1
        elif self.true_valor:
            self.tour_test = 0
        if self.tour_test >= 30:
            self.tour_test = 0
            self.initialisation = False
            self.current_robot = None
            self.previous_robot = None
        if not self.initialisation:
            self.tour_reinit += 1

    # 3
    def _compare_previous_found_positions(self, potential_robot_datas):
        """Compare estimated positions with the previous position obtained."""
        robot_datas = potential_robot_datas.copy()
        dst_min = robot_datas[0]["err"]
        best_match = robot_datas[0]
        for robot_data in robot_datas[1:]:
            dst_test = robot_data["err"]
            if dst_test < dst_min:
                dst_min = dst_test
                best_match = robot_data
        if dt(self.previous_robot["position"], best_match["position"]) < self.precision:
            self.current_robot = best_match
            self.true_valor = True
        else:
            self.true_valor = False
            self.current_robot = self.previous_robot

    # 4
    def _recreate_beacons(self) -> None:
        """Recreate beacons from the known position and beacons in world frame."""
        for i in range(4):
            if self.current_robot["beacons"][i] is None:
                self.current_robot["beacons"][i] = convert_world_to_robot(
                    self.fixed_beacons[i], self.current_robot["position"]
                )

    # 5
    def _find_robots_on_plateau(self, obstacles) -> None:
        """Find other robots on board using its current position."""
        cos_theta = np.cos(self.current_robot["position"][2])
        sin_theta = np.sin(self.current_robot["position"][2])
        OtoR = np.array(
            [
                [cos_theta, -sin_theta, self.current_robot["position"][0]],
                [sin_theta, cos_theta, self.current_robot["position"][1]],
                [0, 0, 1],
            ]
        )
        self.current_robot["other_robots"] = []
        for obstacle in obstacles:
            oP = OtoR @ np.array([obstacle[0], obstacle[1], 1])
            if (
                oP[0] > self.boundaries[0]
                and oP[0] < self.boundaries[1]
                and oP[1] > self.boundaries[2]
                and oP[1] < self.boundaries[3]
            ):
                self.current_robot["other_robots"].append(oP)

    # 2
    def _init_finding(self, potential_robot_datas) -> None:
        """Initialize the position (Fixing)."""
        if len(self.registre_init) == 0:
            liste_tool_globale = []
            liste_balise = potential_robot_datas.copy()
            while len(liste_balise) > 0:
                ver = False
                bal = liste_balise.pop()
                for element in liste_tool_globale:
                    if not ver:
                        if (
                            abs(element[0]["position"][0] - bal["position"][0])
                            < self.precision
                            and abs(element[0]["position"][1] - bal["position"][1])
                            < self.precision
                        ):
                            ver = True
                            element[1] += 1
                            element[0]["position"][0] = (
                                element[0]["position"][0] * (element[1] - 1)
                                + bal["position"][0]
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
                            abs(element[0]["position"][0] - bal["position"][0])
                            < self.precision
                            and abs(element[0]["position"][1] - bal["position"][1])
                            < self.precision
                        ):
                            ver = True
                            element[1] += 1
                            element[0]["position"][0] = (
                                element[0]["position"][0] * (element[1] - 1)
                                + bal["position"][0]
                            ) / element[1]
                if not ver:
                    for element in liste_prev:
                        if not ver:
                            if (
                                abs(element[0]["position"][0] - bal["position"][0])
                                < self.precision
                                and abs(element[0]["position"][1] - bal["position"][1])
                                < self.precision
                            ):
                                ver = True
                                liste_tool_globale.append([bal, 1 + element[1]])
                                removearray(liste_rm, element)
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
                    donnees["position"][2] = find_angle(
                        donnees["position"],
                        donnees["beacons"],
                        self.fixed_beacons,
                    )
                    self.current_robot = donnees
                    self._recreate_beacons()
                    self.true_valor = True
                    self.initialisation = True
            self.tour_reinit = 0
            self.tour_test = 0
            self.registre_init = []

    # Used in beacon_detector_node.py
    def search_pos(self, nb_potential_beacons, potential_beacons, obstacles) -> None:
        """Look for the best estimation of the robot position."""
        self._update_data()
        potential_robot_datas = find_position(
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
                self._recreate_beacons()
                self._find_robots_on_plateau(obstacles)
                return self.current_robot
        self.true_valor = False
        return None
