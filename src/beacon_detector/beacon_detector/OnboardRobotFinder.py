import numpy as np
from .math_lidar import *


class OnboardRobotFinder:

    def __init__(
        self, boundaries, init_position
    ) -> None:
        self.boundaries = boundaries
        self.init_position = Point(x=init_position[0], y=init_position[1], z=np.pi * init_position[2] / 180)

    def get_onboard_robot(self, obstacles):
        robots = []
        for obstacle in obstacles:
            pot_robot = chgt_base_robot_to_plateau(obstacle, self.init_position)
            if (pot_robot.x > self.boundaries[0]) and (pot_robot.x < self.boundaries[1]) and (pot_robot.y > self.boundaries[2]) and (pot_robot.y < self.boundaries[3]):
                robots.append([pot_robot.x, pot_robot.y])
        return robots
