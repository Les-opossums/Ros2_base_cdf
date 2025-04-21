#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from action_sequencer.action_sequencer_node import ActionManager
from action_sequencer.action_manager import Version, Position, Speed
from action_sequencer.action_manager import PUMP_struct, LED_struct
from action_sequencer.action_manager import ODOM_struct, SERVO_struct


class Script(ActionManager):
    def __init__(self):
        super().__init__('script1')
        self.pos_departure = Position(0, 0, 0)

    def run(self):
        self.move_to(self.pos_departure)


if __name__ == "__main__":
    script = Script()
    script.run()
