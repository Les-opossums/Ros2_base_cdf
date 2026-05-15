#!/usr/bin/env python3
# -*- coding: utf-8 -*-


from opossum_action_sequencer.scripts.action_sequencer_node import ActionManager
from opossum_action_sequencer.action_manager import Version, Position, Speed
from opossum_action_sequencer.action_manager import PUMP_struct, LED_struct
from opossum_action_sequencer.action_manager import ODOM_struct, SERVO_struct
import time
import threading


class Script():
    def run(self, node):
        start_match_time = time.time()
        node.send_raw("VMAX 1.0")
        node.move_to(Position(2.7, 0.9), seuil=0.05)
        node.wait_for_motion()

        while time.time() - start_match_time < 10000:
            for k in range(7):
                vmax = 0.2 + k*0.2

                node.send_raw(f"VMAX {vmax}")
                node.move_to(Position(0.5, 0.45), seuil=0.05)
                node.wait_for_motion()

                node.move_to(Position(2.5, 0.45), seuil=0.05)
                node.wait_for_motion()
