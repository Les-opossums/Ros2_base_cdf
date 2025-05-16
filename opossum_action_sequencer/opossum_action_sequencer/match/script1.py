#!/usr/bin/env python3
# -*- coding: utf-8 -*-


from opossum_action_sequencer.scripts.action_sequencer_node import ActionManager
from opossum_action_sequencer.action_manager import Version, Position, Speed
from opossum_action_sequencer.action_manager import PUMP_struct, LED_struct
from opossum_action_sequencer.action_manager import ODOM_struct, SERVO_struct
import time


class Script(ActionManager):
    def __init__(self):
        super().__init__('script1')
        self.pos_departure = Position(0.6, 0.6, 0)

    def run(self):
        self.write_log('Script 1 is running...')
        # self.send_raw('SYNCHROLIDAR')
        # self.send_raw('SETX 0.6')
        # self.send_raw('SETY 0.6')
        # self.send_raw('SETT -1.2')
        self.synchro_lidar()
        time.sleep(2)
        # self.move_to(self.pos_departure)
        for _ in range(5):
            self.add_score(1)
            self.move_to(Position(0.6, 0.6, -1.2))
            self.pump(PUMP_struct(1, 1))
            self.pump(PUMP_struct(2, 1))
            time.sleep(4)
            self.move_to(Position(0.6, 1.45, 0))
            self.pump(PUMP_struct(1, 0))
            self.pump(PUMP_struct(2, 1))
            time.sleep(8)
            self.synchro_lidar()


if __name__ == "__main__":
    script = Script()
    script.run()
