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
        # self.move_to(self.pos_departure)
        for _ in range(10):
            self.move_to(Position(0.6, 0.6, 0))
            time.sleep(10)
            self.move_to(Position(0.6, 1.45, 0))
            time.sleep(10)
        self.write_log('Script 1 is running...')

if __name__ == "__main__":
    script = Script()
    script.run()
