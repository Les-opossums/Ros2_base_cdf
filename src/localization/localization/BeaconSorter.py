from .math_lidar import *

class BeaconSorter():
    def __init__(
        self,
        dst_beacons: dict[str, float],
        sign_vect_product: list[float],
        ang_tol: float,
        dst_tol: float,
    ) -> None:
        """
        This class is used to sort the detected objects and
        find the possible beacons

        :param dst_beacons: The distances between the beacons
        :type dst_beacons: dict
        :param sign_vect_product: The sign of the vectorial product
        between the beacons
        :type sign_vect_product: list
        :param ang_tol: The tolerance on the angle
        :type ang_tol: float
        :param dst_tol: The tolerance on the distance
        :type dst_tol: float
        """
        self._dst_beacons = dst_beacons
        self._sign_vect_product = sign_vect_product
        self._ang_tol = ang_tol
        self._dst_tol = dst_tol

    def _sort_comparison(
        self, beacons: list[np.array], new_objects_detected: list[np.array]
    ) -> dict[str, list[np.array]]:
        """
        This function is used to associate the detected objects
        with the beacons known from past detections

        :param beacons: The beacons
        :type beacons: list
        :param new_objects_detected: The detected objects
        :type new_objects_detected: list

        :return: The sorted objects
        :rtype: dict
        """
        beacons_ang = []
        beacons_dst = []
        for i in range(4):
            a, d = cartesian_to_polar(beacons[i][0], beacons[i][1])
            beacons_ang.append((a - self._ang_tol, a + self._ang_tol))
            beacons_dst.append(d)
        sorting_list = {"A": [], "B": [], "C": [], "D": []}
        for obstacle in new_objects_detected:
            for val, bang, bdst in zip(sorting_list.values(), beacons_ang, beacons_dst):
                angle, distance = cartesian_to_polar(obstacle[0], obstacle[1])
                if abs(distance - bdst) < self._dst_tol and angle_in_range(
                    angle, bang[0], bang[1]
                ):
                    val.append(obstacle)
        return {
            key: [np.array([-100, -100])] if not val else val
            for key, val in sorting_list.items()
        }

    def _find_beacons_prev(
        self, beacons: list[np.array], new_objects_detected: list[np.array]
    ) -> tuple[int, list[list[np.array]]]:
        """
        This function is used to find the possible beacons
        using the previous detected beacons

        :param beacons: The previous detected beacons
        :type beacons: list
        :param new_objects_detected: The detected objects
        :type new_objects_detected: list

        :return: The number of beacons found and the list of the possible beacons
        :rtype: tuple
        """
        sorting_list = self._sort_comparison(beacons, new_objects_detected)
        indexes2, indexes3, indexes4 = [], [], []
        flags = {
            ("A", "B"): False,
            ("A", "C"): False, 
            ("B", "C"): False, 
            ("A", "D"): False, 
            ("B", "D"): False, 
            ("C", "D"): False, 
        }
        for i, bA in enumerate(sorting_list["A"]):
            for j, bB in enumerate(sorting_list["B"]):
                flags[("A", "B")] = False
                if approx(dt(bA, bB), self._dst_beacons[("A", "B")], self._dst_tol):
                    flags[("A", "B")] = True
                    indexes2.append([i, j, -1, -1])
                for k, bC in enumerate(sorting_list["C"]):
                    for key in list(flags.keys())[1:3]:
                        flags[key] = False
                    if approx(dt(bA, bC), self._dst_beacons[("A", "C")], self._dst_tol):
                        flags[("A", "C")] = True
                        indexes2.append([i, -1, k, -1])
                    if approx(dt(bB, bC), self._dst_beacons[("B", "C")], self._dst_tol):
                        flags[("B", "C")] = True
                        indexes2.append([-1, j, k, -1])
                    if flags[("A", "B")] and flags[("A", "C")] and flags[("B", "C")]:
                        indexes3.append([i, j, k, -1])
                    for l, bD in enumerate(sorting_list["D"]):
                        for key in list(flags.keys())[3:]:
                            flags[key] = False
                        if approx(dt(bA, bD), self._dst_beacons[("A", "D")], self._dst_tol):
                            flags[("A", "D")] = True
                            indexes2.append([i, -1, -1, l])
                        if approx(dt(bB, bD), self._dst_beacons[("B", "D")], self._dst_tol):
                            flags[("B", "D")] = True
                            indexes2.append([-1, j, -1, l])
                        if approx(dt(bC, bD), self._dst_beacons[("C", "D")], self._dst_tol):
                            flags[("C", "D")] = True
                            indexes2.append([-1, -1, k, l])
                        if flags[("A", "B")] and flags[("A", "D")] and flags[("B", "D")]:
                            indexes3.append([i, j, -1, l])
                        if flags[("A", "C")] and flags[("A", "D")] and flags[("C", "D")]:
                            indexes3.append([i, -1, k, l])
                        if flags[("B", "C")] and flags[("B", "D")] and flags[("C", "D")]:
                            indexes3.append([-1, j, k, l])
                        if flags[("A", "B")] and flags[("A", "C")] and flags[("B", "C")] and flags[("A", "D")] and flags[("B", "D")] and flags[("C", "D")]:
                            indexes4.append([i, j, k, l])
                            
        if indexes4 != []:
            unique_indexes4 = np.unique(np.array(indexes4), axis=0)
            clean_beacons4 = []
            for index in unique_indexes4:
                clean_beacons4.append([sorting_list[chr(i + 65)][index[i]] for i in range(4)])

            beacons_list = clean_beacons4.copy()
            for beacons in beacons_list:
                sign_res = [get_vp_sign([beacons[i] for i in range(4) if i != 3 - j]) for j in range(4)]
                if sign_res != self._sign_vect_product:
                    removearray(clean_beacons4, beacons)

            if clean_beacons4 != []:
                return 4, clean_beacons4

        if indexes3 != []:
            unique_indexes3 = np.unique(np.array(indexes3), axis=0)
            clean_beacons3 = []
            for index in unique_indexes3:
                clean_beacons3.append([sorting_list[chr(i + 65)][index[i]] if index[i] >= 0 else None for i in range(4)])

            clean_beacons3 = [
                beacons for beacons in clean_beacons3 if get_vp_sign([beacon for beacon in beacons if beacon is not None])
                == self._sign_vect_product[[3 - i for i in range(len(beacons)) if beacons[i] is None][0]]
            ]
            if clean_beacons3 != []:
                return 3, clean_beacons3

        if indexes2 != []:
            unique_indexes2 = np.unique(np.array(indexes2), axis=0)
            clean_beacons2 = []
            for index in unique_indexes2:
                clean_beacons2.append([sorting_list[chr(i + 65)][index[i]] if index[i] >= 0 else None for i in range(4)])
            return 2, clean_beacons2

        else:
            return 0, []

    def _find_beacons_naive(
        self, new_objects_detected: list[np.array]
    ) -> tuple[int, list[list[np.array]]]:
        """
        This function is used to find the possible beacons using a
        naive approach when no past detection is available

        :param new_objects_detected: The detected objects
        :type new_objects_detected: list

        :return: The number of beacons found and the list of the possible beacons
        :rtype: tuple"""

        olist = new_objects_detected.copy()
        beacon_list4, beacon_list3, beacon_list2 = [], [], []
        letters = ['A', 'B', 'C', 'D']
        # While all the ocjects detected by the lidar have not been analysed, we loop
        while len(olist) >= 2:
            # Creation of dictionnary to keep the link between the current object and all the others
            beacon1 = olist.pop()
            dict_indexes = {
                (letters[i], letters[j]): []
                for i in range(4)
                for j in range(i + 1, 4)
            }
            # We compare it to the other ojects and look at the distance between the two 
            # and if corresponding to distance between 2 beacons
            for index, cercle in enumerate(olist):
                distance = dt(beacon1, cercle)
                for key in dict_indexes.keys():
                    if approx(distance, self._dst_beacons[key], self._dst_tol):
                        dict_indexes[key].append(index)
                        duo_beacons = [None] * 4
                        duo_beacons[toint(key[0])] = beacon1
                        duo_beacons[toint(key[1])] = cercle
                        beacon_list2.append(duo_beacons.copy())
                        duo_beacons[toint(key[1])] = beacon1
                        duo_beacons[toint(key[0])] = cercle
                        beacon_list2.append(duo_beacons.copy())
            # We now will study our current object as each of the beacon.
            for clet in letters:
                # Here are the other obejct we will consider as other beacons and study their distances
                olet = [j for j in letters if j != clet]
                for i in range(3):
                    for ind1 in dict_indexes[sorttuple(clet, olet[i])]:
                        for ind2 in dict_indexes[sorttuple(clet, olet[(i + 1) % 3])]:
                            # Loop now to see if 3 points can correspond to beacons. For example before we considered the object with others
                            # Lets suppose the current object is considered beacon A, Maybe it had links with another as B, and another as D. 
                            # We now compute the distance between the "considered B beacon" and "D" one. If the "BD distance" is right and the orientation of the angle is right, 
                            # we save the 3 beacons as possible beacons. 
                            if approx(dt(olist[ind1], olist[ind2]), self._dst_beacons[sorttuple(olet[i], olet[(i + 1) % 3])], self._dst_tol):
                                temp_list = [None] * 4
                                temp_list[toint(clet)] = beacon1
                                temp_list[toint(olet[i])] = olist[ind1]
                                temp_list[toint(olet[(i + 1) % 3])] = olist[
                                    ind2
                                ]
                                none_index = next((i for i, x in enumerate(temp_list) if x is None), None)
                                if get_vp_sign([b for b in temp_list if b is not None]) == self._sign_vect_product[3 - none_index]:
                                    beacon_list3.append(temp_list.copy())
                                if i == 0:
                                    # Here, we chack if 4 objects can represent the 4 beacons. That is the reason why we only iterate for the first A beacon. 
                                    # Because lets suppose we have 4 objects, we would have 3 times the same 4-beacon combination so we do it only for the first one. 
                                    for ind3 in dict_indexes[sorttuple(clet, olet[(i + 2) % 3])]:
                                        if approx(dt(olist[ind1], olist[ind3]), self._dst_beacons[sorttuple(olet[i], olet[(i + 2) % 3])], self._dst_tol) \
                                            and approx(dt(olist[ind2], olist[ind3]), self._dst_beacons[sorttuple(olet[(i + 1) % 3], olet[(i + 2) % 3])], self._dst_tol):
                                            temp_list[toint(olet[(i + 2) % 3])] = olist[ind3]
                                            sign_res = [get_vp_sign([temp_list[p] for p in range(4) if p != 3 - m]) for m in range(4)]
                                            if sign_res == self._sign_vect_product:
                                                beacon_list4.append(temp_list.copy())
        if beacon_list4 != []:
            return 4, beacon_list4

        elif beacon_list3 != []:
            return 3, beacon_list3

        elif beacon_list2 != []:
            return 2, beacon_list2

        else:
            return 0, []

    def _find_possible_beacons(
        self, previous_beacons: list[np.array], new_objects_detected: list[np.array]
    ) -> tuple[int, list[list[np.array]]]:
        """
        This function is used to find the possible beacons

        :param previous_beacons: The previous detected beacons
        :type previous_beacons: list
        :param new_objects_detected: The detected objects
        :type new_objects_detected: list

        :return: The number of beacons found and the list of the possible beacons
        :rtype: tuple
        """
        if previous_beacons is not None:
            beacons_data = self._find_beacons_prev(
                previous_beacons, new_objects_detected
            )
            if beacons_data[0] > 2:
                return beacons_data
        beacons_data = self._find_beacons_naive(new_objects_detected)
        return beacons_data
