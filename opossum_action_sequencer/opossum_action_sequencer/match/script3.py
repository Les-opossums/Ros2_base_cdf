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
            node.write_log('Script 3 is running...')
            # self.send_raw('SYNCHROLIDAR')
            # self.send_raw('SETX 0.6')
            # self.send_raw('SETY 0.6')
            # self.send_raw('SETT -1.2')
            # node.synchro_lidar()
            # self.move_to(self.pos_departure)
            node.send_raw(f'VMAX 0.7')
            time.sleep(2)
            node.move_to(Position(0.5, 1.32, 0.54))
            node.wait_for_motion()
            time.sleep(2)
            node.send_raw(f'VMAX 0.2')
            node.move_to(Position(0.1, 1.32, 0.54))
            node.wait_for_motion()
            node.pump(PUMP_struct(1, 1))
            node.pump(PUMP_struct(2, 1))


            node.move_to(Position(0.3, 1.32, 0.54))
            node.wait_for_motion()
            node.move_to(Position(0.35, 1.7, -0.95))
            node.wait_for_motion()
            node.pump(PUMP_struct(1, 0))
            node.pump(PUMP_struct(2, 0))
            node.add_score(10)

            node.send_raw(f'VMAX 0.7')
            node.move_to(Position(0.35, 1.55, -0.95))
            node.wait_for_motion()
            node.move_to(Position(1.05, 0.37, -2.6))
            node.wait_for_motion()

    def stop(self):
        self._stop_event.set()
