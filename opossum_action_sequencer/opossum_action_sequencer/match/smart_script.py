#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
available_cans = {i: True for i in range(16)}

position_cans = {
    0: [0.075, 1.325, 1],
    1: [0.075, 0.4, 1],
    2: [0.825, 1.725, 0],
    3: [1.1, 0.95, 0],
    4: [0.775, 0.25, 0],
}

position_cans = position_cans | {key + 5: [3 - val[0], val[1], val[2]] for key, val in position_cans.items()}
end_poses = {
    
}
from opossum_action_sequencer.scripts.action_sequencer_node import ActionManager
from opossum_action_sequencer.action_manager import Version, Position, Speed
from opossum_action_sequencer.action_manager import PUMP_struct, LED_struct
from opossum_action_sequencer.action_manager import ODOM_struct, SERVO_struct
from opossum_action_sequencer.action_manager import STEPPER_struct
from opossum_action_sequencer.action_manager import VALVE_struct
import time
from rclpy.logging import get_logger


class Script:
    def __init__(self):
        self.id_mvt = 0

    def run(self, node):
        node.write_log("Script GNAAAAAAAAAAAAAais running...")

        # Banderole
        node.send_raw("VMAX 0.5")
        node.smart_moves()