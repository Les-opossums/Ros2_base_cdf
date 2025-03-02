import numpy as np
from .math_lidar import *


class OnboardRobotFinder:

    def __init__(
        self, boundaries, init_position
    ) -> None:
        self.boundaries = boundaries
        self.init_position = np.array(
            [init_position[0], init_position[1], np.pi * init_position[2] / 180]
        )

    def get_onboard_robot(self, new_objects):
        objects = []
        for new_obj in new_objects:
            obj = chgt_base_robot_to_plateau(new_obj, self.init_position)
            if (
                (obj[0] > self.boundaries[0])
                and (obj[0] < self.boundaries[1])
                and (obj[1] > self.boundaries[2])
                and (obj[1] < self.boundaries[3])
            ):
                objects.append(obj)
        return objects
