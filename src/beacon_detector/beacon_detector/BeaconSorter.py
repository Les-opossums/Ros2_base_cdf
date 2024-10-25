from .objet import RobotDatas
from .math_lidar import *


class BeaconSorter:

    def __init__(self, dst_beacons, sign_vect_product, ang_tol, dst_tol) -> None:
        self._dst_beacons = dst_beacons
        self._sign_vect_product = sign_vect_product
        self._ang_tol = ang_tol
        self._dst_tol = dst_tol

    def _sort_comparison(self, beacons, new_objects_detected) -> bool:
        beacons_ang = []
        beacons_dst = []
        for i in range(4):
            a, d = cartesian_to_polar(beacons[i].x, beacons[i].y)
            beacons_ang.append((a - self._ang_tol, a + self._ang_tol))
            beacons_dst.append(d)
        sorting_list = {"A": [], "B": [], "C": [], "D": []}
        for obstacle in new_objects_detected:
            for i, key in enumerate(sorting_list.keys()):
                angle, distance = cartesian_to_polar(obstacle.x, obstacle.y)
                if abs(distance - beacons_dst[i]) < self._dst_tol and angle_in_range(
                    angle, beacons_ang[i][0], beacons_ang[i][1]
                ):
                    sorting_list[key].append(obstacle)
        return sorting_list

    def _find_beacons_prev(self, beacons, new_objects_detected) -> list[RobotDatas]:
        sorting_list = self._sort_comparison(beacons, new_objects_detected)
        pot_balise_2, pot_balise_3, pot_balise_4 = [], [], []
        for i, baliseA in enumerate(sorting_list["A"]):
            for j, baliseB in enumerate(sorting_list["B"]):
                for baliseC in sorting_list["C"]:
                    for baliseD in sorting_list["D"]:
                        (
                            value_AB,
                            value_AC,
                            value_AD,
                            value_BC,
                            value_BD,
                            value_CD,
                        ) = (False, False, False, False, False, False)
                        if approx(
                            dt(baliseA, baliseB),
                            self._dst_beacons["AB"],
                            self._dst_tol,
                        ):
                            value_AB = True
                            pot_balise_2.append(
                                RobotDatas(baliseA, baliseB, None, None)
                            )
                        if approx(
                            dt(baliseA, baliseC),
                            self._dst_beacons["AC"],
                            self._dst_tol,
                        ):
                            value_AC = True
                            pot_balise_2.append(
                                RobotDatas(baliseA, None, baliseC, None)
                            )
                        if approx(
                            dt(baliseA, baliseD),
                            self._dst_beacons["AD"],
                            self._dst_tol,
                        ):
                            value_AD = True
                            pot_balise_2.append(
                                RobotDatas(baliseA, None, None, baliseD)
                            )
                        if approx(
                            dt(baliseB, baliseC),
                            self._dst_beacons["BC"],
                            self._dst_tol,
                        ):
                            value_BC = True
                            if i == 0:
                                pot_balise_2.append(
                                    RobotDatas(None, baliseB, baliseC, None)
                                )
                        if approx(
                            dt(baliseB, baliseD),
                            self._dst_beacons["BD"],
                            self._dst_tol,
                        ):
                            value_BD = True
                            if i == 0:
                                pot_balise_2.append(
                                    RobotDatas(None, baliseB, None, baliseD)
                                )
                        if approx(
                            dt(baliseC, baliseD),
                            self._dst_beacons["CD"],
                            self._dst_tol,
                        ):
                            value_CD = True
                            if i == 0 and j==0:
                                pot_balise_2.append(
                                    RobotDatas(None, None, baliseC, baliseD)
                                )
                        if value_AB and value_AC and value_BC:
                            pot_balise_3.append(
                                RobotDatas(baliseA, baliseB, baliseC, None)
                            )
                        if value_AB and value_AD and value_BD:
                            pot_balise_3.append(
                                RobotDatas(baliseA, baliseB, None, baliseD)
                            )
                        if value_AC and value_AD and value_CD:
                            pot_balise_3.append(
                                RobotDatas(baliseA, None, baliseC, baliseD)
                            )
                        if value_BC and value_BD and value_CD and i==0:
                            pot_balise_3.append(
                                RobotDatas(None, baliseB, baliseC, baliseD)
                            )
                        if value_AB and value_AC and value_AD and value_BC:
                            pot_balise_4.append(
                                RobotDatas(baliseA, baliseB, baliseC, baliseD)
                            )
        if pot_balise_4 != []:
            pot_bal_copy = pot_balise_4.copy()
            for pot in pot_bal_copy:
                pot_sign_vect_product = []
                pot_sign_vect_product = [
                    get_sign_vect_product(pot.balise_A, pot.balise_B, pot.balise_C), 
                    get_sign_vect_product(pot.balise_A, pot.balise_B, pot.balise_D),
                    get_sign_vect_product(pot.balise_A, pot.balise_C, pot.balise_D),
                    get_sign_vect_product(pot.balise_B, pot.balise_C, pot.balise_D),
                ]
                if pot_sign_vect_product != self._sign_vect_product:
                    pot_balise_4.remove(pot)
            if pot_balise_4 != []:
                return 4, pot_balise_4

        if pot_balise_3 != []:
            pot_bal_copy = pot_balise_3.copy()
            for pot in pot_bal_copy:
                if pot.balise_D is None:
                    pot_sign_vect_product = get_sign_vect_product(pot.balise_A, pot.balise_B, pot.balise_C)
                    if pot_sign_vect_product != self._sign_vect_product[0]:
                        pot_balise_3.remove(pot)
                elif pot.balise_C is None:
                    pot_sign_vect_product = get_sign_vect_product(pot.balise_A, pot.balise_B, pot.balise_D)
                    if pot_sign_vect_product != self._sign_vect_product[1]:
                        pot_balise_3.remove(pot)
                elif pot.balise_B is None:
                    pot_sign_vect_product = get_sign_vect_product(pot.balise_A, pot.balise_C, pot.balise_D)
                    if pot_sign_vect_product != self._sign_vect_product[2]:
                        pot_balise_3.remove(pot)
                else:
                    pot_sign_vect_product = get_sign_vect_product(pot.balise_B, pot.balise_C, pot.balise_D)
                    if pot_sign_vect_product != self._sign_vect_product[3]:
                        pot_balise_3.remove(pot)
            if pot_balise_3 != []:
                return 3, pot_balise_3

        if pot_balise_2 != []:
            return 2, pot_balise_2

        else:
            return 0, []

    def _find_beacons_naive(self, new_objects_detected) -> list[RobotDatas]:
        liste_obstacle = new_objects_detected.copy()
        pot_balise_4, pot_balise_3, pot_balise_2 = [], [], []
        while len(liste_obstacle) >= 2:
            (
                index_to_verif_AB,
                index_to_verif_AC,
                index_to_verif_AD,
                index_to_verif_BC,
                index_to_verif_BD,
                index_to_verif_CD,
            ) = ([], [], [], [], [], [])
            balise1 = liste_obstacle.pop()
            for index, cercle in enumerate(liste_obstacle):
                distance = dt(balise1, cercle)
                if approx(distance, self._dst_beacons["AB"], self._dst_tol):
                    index_to_verif_AB.append(index)
                    pot_balise_2.append(RobotDatas(balise1, cercle, None, None))
                    pot_balise_2.append(RobotDatas(cercle, balise1, None, None))
                if approx(distance, self._dst_beacons["AC"], self._dst_tol):
                    index_to_verif_AC.append(index)
                    pot_balise_2.append(RobotDatas(balise1, None, cercle, None))
                    pot_balise_2.append(RobotDatas(cercle, None, balise1, None))
                if approx(distance, self._dst_beacons["AD"], self._dst_tol):
                    index_to_verif_AD.append(index)
                    pot_balise_2.append(RobotDatas(balise1, None, None, cercle))
                    pot_balise_2.append(RobotDatas(cercle, None, None, balise1))
                if approx(distance, self._dst_beacons["BC"], self._dst_tol):
                    index_to_verif_BC.append(index)
                    pot_balise_2.append(RobotDatas(None, balise1, cercle, None))
                    pot_balise_2.append(RobotDatas(None, cercle, balise1, None))
                if approx(distance, self._dst_beacons["BD"], self._dst_tol):
                    index_to_verif_BD.append(index)
                    pot_balise_2.append(RobotDatas(balise1, None, None, cercle))
                    pot_balise_2.append(RobotDatas(cercle, None, None, balise1))
                if approx(distance, self._dst_beacons["CD"], self._dst_tol):
                    index_to_verif_CD.append(index)
                    pot_balise_2.append(RobotDatas(None, None, cercle, balise1))
                    pot_balise_2.append(RobotDatas(None, None, balise1, cercle))
            for i in index_to_verif_AB:
                for j in index_to_verif_AC:
                    value_BC = False
                    if approx(
                        dt(new_objects_detected[i], new_objects_detected[j]),
                        self._dst_beacons["BC"],
                        self._dst_tol,
                    ):
                        value_BC = True
                        pot_balise_3.append(
                            RobotDatas(
                                balise1,
                                new_objects_detected[i],
                                new_objects_detected[j],
                                None,
                            )
                        )
                    for k in index_to_verif_AD:
                        value_BD = False
                        value_CD = False
                        if approx(
                            dt(new_objects_detected[i], new_objects_detected[k]),
                            self._dst_beacons["BD"],
                            self._dst_tol,
                        ):
                            value_BD = True
                            pot_balise_3.append(
                                RobotDatas(
                                    balise1,
                                    new_objects_detected[i],
                                    None,
                                    new_objects_detected[k],
                                )
                            )
                        if approx(
                            dt(new_objects_detected[j], new_objects_detected[k]),
                            self._dst_beacons["CD"],
                            self._dst_tol,
                        ):
                            value_CD = True
                            pot_balise_3.append(
                                RobotDatas(
                                    balise1,
                                    None,
                                    new_objects_detected[j],
                                    new_objects_detected[k],
                                )
                            )
                        if value_BC and value_BD and value_CD:
                            pot_balise_4.append(
                                RobotDatas(
                                    balise1,
                                    new_objects_detected[i],
                                    new_objects_detected[j],
                                    new_objects_detected[k],
                                )
                            )

                for j in index_to_verif_BC:
                    value_AC = False
                    if approx(
                        dt(new_objects_detected[i], new_objects_detected[j]),
                        self._dst_beacons["AC"],
                        self._dst_tol,
                    ):
                        value_AC = True
                        pot_balise_3.append(
                            RobotDatas(
                                new_objects_detected[i],
                                balise1,
                                new_objects_detected[j],
                                None,
                            )
                        )
                    for k in index_to_verif_BD:
                        value_AD = False
                        value_CD = False
                        if approx(
                            dt(new_objects_detected[i], new_objects_detected[k]),
                            self._dst_beacons["AD"],
                            self._dst_tol,
                        ):
                            value_AD = True
                            pot_balise_3.append(
                                RobotDatas(
                                    new_objects_detected[i],
                                    balise1,
                                    None,
                                    new_objects_detected[k],
                                )
                            )
                        if approx(
                            dt(new_objects_detected[j], new_objects_detected[k]),
                            self._dst_beacons["CD"],
                            self._dst_tol,
                        ):
                            value_CD = True
                            pot_balise_3.append(
                                RobotDatas(
                                    None,
                                    balise1,
                                    new_objects_detected[j],
                                    new_objects_detected[k],
                                )
                            )
                        if value_AC and value_AD and value_CD:
                            pot_balise_4.append(
                                RobotDatas(
                                    new_objects_detected[i],
                                    balise1,
                                    new_objects_detected[j],
                                    new_objects_detected[k],
                                )
                            )

            for i in index_to_verif_CD:
                for j in index_to_verif_AC:
                    value_DA = False
                    if approx(
                        dt(new_objects_detected[i], new_objects_detected[j]),
                        self._dst_beacons["AD"],
                        self._dst_tol,
                    ):
                        value_DA = True
                        pot_balise_3.append(
                            RobotDatas(
                                new_objects_detected[j],
                                None,
                                balise1,
                                new_objects_detected[i],
                            )
                        )
                    for k in index_to_verif_BC:
                        value_DB = False
                        value_AB = False
                        if approx(
                            dt(new_objects_detected[i], new_objects_detected[k]),
                            self._dst_beacons["BD"],
                            self._dst_tol,
                        ):
                            value_DB = True
                            pot_balise_3.append(
                                RobotDatas(
                                    None,
                                    new_objects_detected[k],
                                    balise1,
                                    new_objects_detected[i],
                                )
                            )
                        if approx(
                            dt(new_objects_detected[j], new_objects_detected[k]),
                            self._dst_beacons["AB"],
                            self._dst_tol,
                        ):
                            value_AB = True
                            pot_balise_3.append(
                                RobotDatas(
                                    new_objects_detected[j],
                                    new_objects_detected[k],
                                    balise1,
                                    None,
                                )
                            )
                        if value_DA and value_DB and value_AB:
                            pot_balise_4.append(
                                RobotDatas(
                                    new_objects_detected[j],
                                    new_objects_detected[k],
                                    balise1,
                                    new_objects_detected[i],
                                )
                            )

                for j in index_to_verif_AD:
                    value_CA = False
                    if approx(
                        dt(new_objects_detected[i], new_objects_detected[j]),
                        self._dst_beacons["AC"],
                        self._dst_tol,
                    ):
                        value_CA = True
                        pot_balise_3.append(
                            RobotDatas(
                                new_objects_detected[j],
                                None,
                                new_objects_detected[i],
                                balise1,
                            )
                        )
                    for k in index_to_verif_BD:
                        value_CB = False
                        value_AB = False
                        if approx(
                            dt(new_objects_detected[i], new_objects_detected[k]),
                            self._dst_beacons["BC"],
                            self._dst_tol,
                        ):
                            value_CB = True
                            pot_balise_3.append(
                                RobotDatas(
                                    None,
                                    new_objects_detected[k],
                                    new_objects_detected[i],
                                    balise1,
                                )
                            )
                        if approx(
                            dt(new_objects_detected[j], new_objects_detected[k]),
                            self._dst_beacons["AB"],
                            self._dst_tol,
                        ):
                            value_AB = True
                            pot_balise_3.append(
                                RobotDatas(
                                    new_objects_detected[j],
                                    new_objects_detected[k],
                                    None,
                                    balise1,
                                )
                            )
                        if value_CA and value_CB and value_AB:
                            pot_balise_4.append(
                                RobotDatas(
                                    new_objects_detected[j],
                                    new_objects_detected[k],
                                    new_objects_detected[i],
                                    balise1,
                                )
                            )
        if pot_balise_4 != []:
            pot_bal_copy = pot_balise_4.copy()
            for pot in pot_bal_copy:
                pot_sign_vect_product = []
                pot_sign_vect_product = [
                    get_sign_vect_product(pot.balise_A, pot.balise_B, pot.balise_C), 
                    get_sign_vect_product(pot.balise_A, pot.balise_B, pot.balise_D),
                    get_sign_vect_product(pot.balise_A, pot.balise_C, pot.balise_D),
                    get_sign_vect_product(pot.balise_B, pot.balise_C, pot.balise_D),
                ]
                if pot_sign_vect_product != self._sign_vect_product:
                    pot_balise_4.remove(pot)
            if pot_balise_4 != []:
                return 4, pot_balise_4

        if pot_balise_3 != []:
            pot_bal_copy = pot_balise_3.copy()
            for pot in pot_bal_copy:
                if pot.balise_D is None:
                    pot_sign_vect_product = get_sign_vect_product(pot.balise_A, pot.balise_B, pot.balise_C)
                    if pot_sign_vect_product != self._sign_vect_product[0]:
                        pot_balise_3.remove(pot)
                elif pot.balise_C is None:
                    pot_sign_vect_product = get_sign_vect_product(pot.balise_A, pot.balise_B, pot.balise_D)
                    if pot_sign_vect_product != self._sign_vect_product[1]:
                        pot_balise_3.remove(pot)
                elif pot.balise_B is None:
                    pot_sign_vect_product = get_sign_vect_product(pot.balise_A, pot.balise_C, pot.balise_D)
                    if pot_sign_vect_product != self._sign_vect_product[2]:
                        pot_balise_3.remove(pot)
                else:
                    pot_sign_vect_product = get_sign_vect_product(pot.balise_B, pot.balise_C, pot.balise_D)
                    if pot_sign_vect_product != self._sign_vect_product[3]:
                        pot_balise_3.remove(pot)
            if pot_balise_3 != []:
                return 3, pot_balise_3

        if pot_balise_2 != []:
            return 2, pot_balise_2
        else:
            return 0, []

    def _find_possible_beacons(self, previous_beacons, new_objects_detected) -> None:
        if previous_beacons is not None:
            nb, beacons = self._find_beacons_prev(
                previous_beacons, new_objects_detected
            )
            if nb == 0:
                nb, beacons = self._find_beacons_naive(new_objects_detected)
            return nb, beacons
        else:
            nb, beacons = self._find_beacons_naive(new_objects_detected)
            return nb, beacons
