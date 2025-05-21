#!/usr/bin/env python3
# -*- coding: utf-8 -*-


from opossum_action_sequencer.scripts.action_sequencer_node import ActionManager
from opossum_action_sequencer.action_manager import Version, Position, Speed
from opossum_action_sequencer.action_manager import PUMP_struct, LED_struct
from opossum_action_sequencer.action_manager import ODOM_struct, SERVO_struct
# from opossum_action_sequencer.action_manager import STEPPER_struct
import time
import threading


class Script():
    def __init__(self):
        self._stop_event = threading.Event()

    def run(self, node):
        while not self._stop_event.is_set():
            node.write_log('Script 5 is running...')

            # node.stepper(STEPPER_struct(2))
            node.pump(PUMP_struct(3, 1))
            # node.send_raw('SERVO 1 180')
            time.sleep(2)

            node.pump(PUMP_struct(3, 0))
            # node.stepper(STEPPER_struct(1))
            node.add_score(10)


            # break

    def stop(self):
        self._stop_event.set()
