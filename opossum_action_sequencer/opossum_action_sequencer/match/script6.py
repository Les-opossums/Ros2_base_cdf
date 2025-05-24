#!/usr/bin/env python3
# -*- coding: utf-8 -*-


from opossum_action_sequencer.scripts.action_sequencer_node import ActionManager
from opossum_action_sequencer.action_manager import Version, Position, Speed
from opossum_action_sequencer.action_manager import PUMP_struct, LED_struct
from opossum_action_sequencer.action_manager import ODOM_struct, SERVO_struct
import time
import threading


class Script():
    def __init__(self):
        self._stop_event = threading.Event()

    def run(self, node):
        while not self._stop_event.is_set():
            for _ in range(10):
                node.send_raw('VMAX 0.6')
                node.move_to(Position(0.6, 0.6, 0))
                node.wait_for_motion()
                node.add_score(1)
                node.send_raw('VMAX 0.6')
                node.move_to(Position(2.4, 0.6, 0))
                node.wait_for_motion()
                node.add_score(1)

            break

    def stop(self):
        self._stop_event.set()
