import time
import numpy as np

class RobotState:
    def __init__(self, is_random, linear_velocity, angular_velocity, boundaries):
        self._randomize_state()
        self.linear_velocity = linear_velocity
        self.angular_velocity = angular_velocity
        self.boundaries = boundaries
        self.move = self._random_move if is_random else self._ordered_move
        self.servos_velocity = 0.1
        
    def _randomize_state(self):
        """Create random positions for init."""
        self.angle = np.random.uniform(0, 2 * np.pi)
        self.position = np.random.uniform(0, 2, (2, 1))
        self.servos = {
            0: np.random.uniform(0, np.pi),
            1: np.random.uniform(0, np.pi),
            2: np.random.uniform(0, np.pi),
        }
        self.buttons = {
            "orange": False, 
            "green": False,
            "blue": False,
        }

    def _random_move(self):
        """Do some random moves."""
        self.angle += np.random.uniform(-0.3, 0.3)
        pot_pos = self.position + np.random.uniform(-0.05, 0.05, (2, 1))
        while (
            pot_pos[0] > self.boundaries[1]
            or pot_pos[0] < self.boundaries[0]
            or pot_pos[1] > self.boundaries[3]
            or pot_pos[1] < self.boundaries[2]
        ):
            pot_pos = self.position[:2, :] + np.random.uniform(-0.05, 0.05, (2, 1))
        self.position[:2, :] = pot_pos

    def _ordered_move(self, obj, real_time):
        """Do straight moves when there is a request."""
        real_delta = time.time() - real_time
        deltas = obj[:2, 0] - self.position[:2, 0]
        dst = np.linalg.norm(deltas)
        dangle = obj[2] % (2 * np.pi) - self.angle % (2 * np.pi)
        va = self.angular_velocity if dangle >= 0 else -self.angular_velocity
        if dst < self.linear_velocity * real_delta:
            self.position[:2, 0] = obj[:2, 0]
        else:
            self.position[:2, 0] += real_delta * self.linear_velocity * deltas / dst
        self.angle = (
            self.angle + real_delta * va
            if abs(real_delta * va) < abs(dangle)
            else float(obj[2].item())
        )

    def move_servo(self, id, obj, real_time):
        delta = obj - self.servos[id]
        real_delta = time.time() - real_time
        if real_delta * self.servos_velocity > abs(delta):
            self.servos[id] = obj
        else:
            sign = -1 if delta < 0 else 1
            self.servos += sign * real_delta * self.servos_velocity


        