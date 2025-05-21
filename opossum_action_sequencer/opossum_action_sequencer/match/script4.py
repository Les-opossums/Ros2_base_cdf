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
            node.write_log('Script 4 is running...')
            node.send_raw('VMAX 0.7')
            time.sleep(2)
            node.move_to(Position(1.1, 0.7, -1.12))
            node.wait_for_motion()
            time.sleep(2)
            node.send_raw('VMAX 0.2')
            time.sleep(2)
            # node.send_raw('SERVO 1 20')
            node.pump(PUMP_struct(1, 1))
            node.pump(PUMP_struct(1, 1))
            node.send_raw('PUMP 1 1')
            node.send_raw('PUMP 2 1')
            node.pump(PUMP_struct(2, 1))
            node.pump(PUMP_struct(3, 1))
            node.move_to(Position(1.1, 0.9, -1.12), seuil=0.05)
            node.wait_for_motion()
            # node.stepper(STEPPER_struct(2))
            node.pump(PUMP_struct(3, 1))
            # node.send_raw('SERVO 1 180')
            time.sleep(2)
            # node.stepper(STEPPER_struct(1))

            node.move_to(Position(0.35, 1.7, -0.95))
            node.wait_for_motion()
            # node.stepper(STEPPER_struct(2))
            time.sleep(2)
            node.pump(PUMP_struct(1, 0))
            node.pump(PUMP_struct(2, 0))
            node.pump(PUMP_struct(3, 0))
            # node.stepper(STEPPER_struct(1))
            node.add_score(10)

            node.send_raw('VMAX 0.7')
            node.move_to(Position(0.35, 1.55, -0.95))
            node.wait_for_motion()
            node.move_to(Position(1.05, 0.37, -2.6))
            node.wait_for_motion()

            # break

    def stop(self):
        self._stop_event.set()
