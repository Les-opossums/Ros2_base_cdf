#!/usr/bin/env python3
# -*- coding: utf-8 -*-


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

    def log_mvt(self, node):
        node.write_log(f"Mouvement ID: {self.id_mvt}")
        self.id_mvt += 1

    def run(self, node):
        node.write_log("Script PD is running...")

        for _ in range(20):
            node.send_raw("VMAX 1.5")
            node.move_to(Position(0.6, 0.5, -2.5))
            node.wait_for_motion()
            node.sleep(0.5)

            node.send_raw("VMAX 1.5")
            node.move_to(Position(2.4, 0.5, -2.5))
            node.wait_for_motion()
            node.sleep(0.5)
