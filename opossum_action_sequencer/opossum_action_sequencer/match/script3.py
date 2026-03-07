#!/usr/bin/env python3
# -*- coding: utf-8 -*-


from opossum_action_sequencer.scripts.action_sequencer_node import ActionManager
from opossum_action_sequencer.action_manager import Position, VACCUMGRIPPER_struct
from rclpy.logging import get_logger


class Script:
    def __init__(self):
        self.id_mvt = 0

    def log_mvt(self, node):
        node.write_log(f"Mouvement ID: {self.id_mvt}")
        self.id_mvt += 1

    def run(self, node):
        node.write_log("Script 3 is running...")
        for k in range(20):
                node.send_raw("VMAX 0.3")
                node.move_to(Position(0.5, 1., 0.))
                node.wait_for_motion()
                node.move_to(Position(0.5, 0.5, 0.))
                node.wait_for_motion()