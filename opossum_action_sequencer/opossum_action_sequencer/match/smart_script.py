#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np

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
        node.send_raw("VMAX 0.7")
        node.send_raw("VTMAX 1.0")
        node.smart_moves()