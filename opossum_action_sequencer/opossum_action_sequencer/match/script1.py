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
            speed = [0.1, 1., 0.2, 0.5, 0.7]
            node.write_log('Script 1 is running...')
            # self.send_raw('SYNCHROLIDAR')
            # self.send_raw('SETX 0.6')
            # self.send_raw('SETY 0.6')
            # self.send_raw('SETT -1.2')
            node.synchro_lidar()
            time.sleep(2)
            # self.move_to(self.pos_departure)
            for k in range(5):
                node.add_score(1)
                node.send_raw(f'VMAX {speed[k]}')
                node.move_to(Position(0.6, 0.6, -1.2))
                node.pump(PUMP_struct(1, 1))
                node.pump(PUMP_struct(2, 1))
                node.add_score(10)
                node.wait_for_motion()
                # time.sleep(4)
                node.move_to(Position(0.6, 1.45, 0))
                node.pump(PUMP_struct(1, 0))
                node.pump(PUMP_struct(2, 1))
                node.add_score(10)
                node.wait_for_motion()
                # time.sleep(4)
                node.synchro_lidar()

    def stop(self):
        self._stop_event.set()
