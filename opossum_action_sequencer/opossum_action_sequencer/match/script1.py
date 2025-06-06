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
        node.write_log('Script 1 is running...')
        node.end_zone = Position(1.5, 1, 0)
        # self.send_raw('SYNCHROLIDAR')
        # self.send_raw('SETX 0.6')
        # self.send_raw('SETY 0.6')
        # self.send_raw('SETT -1.2')
        # node.kalman(False)
        node.sleep(2)
        # self.move_to(self.pos_departure)
        for _ in range(20):
            node.add_score(1)
            node.move_to(Position(0.6, 0.6, -1.2))
            node.pump(PUMP_struct(1, 1))
            node.pump(PUMP_struct(2, 1))
            node.add_score(10)
            node.wait_for_motion()
            # node.sleep(4)
            node.move_to(Position(0.6, 1.1, 0))
            node.pump(PUMP_struct(1, 0))
            node.pump(PUMP_struct(2, 1))
            node.add_score(10)
            node.wait_for_motion()
            # node.sleep(4)
            node.synchro_lidar()
            node.move_to(Position(1., 1.1, 0))
            node.pump(PUMP_struct(1, 0))
            node.pump(PUMP_struct(2, 1))
            node.add_score(10)
            node.wait_for_motion()
            # node.sleep(4)
            node.move_to(Position(1., 0.6, 0))
            node.pump(PUMP_struct(1, 0))
            node.pump(PUMP_struct(2, 1))
            node.add_score(10)
            node.wait_for_motion()
            # node.sleep(4)
            node.synchro_lidar()
