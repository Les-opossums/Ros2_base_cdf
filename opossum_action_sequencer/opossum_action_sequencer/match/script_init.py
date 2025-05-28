#!/usr/bin/env python3
# -*- coding: utf-8 -*-


from opossum_action_sequencer.scripts.action_sequencer_node import ActionManager
from opossum_action_sequencer.action_manager import Version, Position, Speed
from opossum_action_sequencer.action_manager import PUMP_struct, LED_struct
from opossum_action_sequencer.action_manager import ODOM_struct, SERVO_struct
from opossum_action_sequencer.action_manager import STEPPER_struct
from opossum_action_sequencer.action_manager import VALVE_struct
import time
import threading


class Script():
    def run(self, node):
        node.write_log('Script Init is running...')
        time.sleep(1)
        node.stepper(STEPPER_struct(0))
        time.sleep(0.5)
        node.stepper(STEPPER_struct(2))
        time.sleep(0.5)
        node.stepper(STEPPER_struct(1))
        time.sleep(0.5)
        node.servo(SERVO_struct(1, 180))
        time.sleep(0.5)
        node.servo(SERVO_struct(1, 20))
        time.sleep(0.5)
        node.servo(SERVO_struct(1, 180))
        time.sleep(0.5)
        node.servo(SERVO_struct(2, 20))
        time.sleep(0.5)
        node.servo(SERVO_struct(2, 180))
        time.sleep(0.5)
        node.servo(SERVO_struct(2, 20))
        time.sleep(0.5)
        node.send_raw('PUMP 1 1')
        time.sleep(1)
        node.send_raw('PUMP 1 0')
        time.sleep(1)
        node.send_raw('PUMP 2 1')
        time.sleep(1)
        node.send_raw('PUMP 2 0')
        time.sleep(1)
        node.send_raw('PUMP 3 1')
        time.sleep(1)
        node.send_raw('PUMP 3 0')
        time.sleep(1)
        node.send_raw('PUMP 4 1')
        time.sleep(1)
        node.send_raw('PUMP 4 0')
        time.sleep(1)
        node.relative_move_to(Position(0, 0, 3))
        node.send_raw('FREE')
        time.sleep(0.5)
        node.relative_move_to(Position(0, 0, 3))
        node.wait_for_motion()
        node.write_log('Script Init completed.')
