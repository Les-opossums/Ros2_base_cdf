#!/usr/bin/env python3
# -*- coding: utf-8 -*-


from opossum_action_sequencer.scripts.action_sequencer_node import ActionManager
from opossum_action_sequencer.action_manager import Version, Position, Speed
from opossum_action_sequencer.action_manager import PUMP_struct, LED_struct
from opossum_action_sequencer.action_manager import ODOM_struct, SERVO_struct
from opossum_action_sequencer.action_manager import STEPPER_struct
from opossum_action_sequencer.action_manager import VALVE_struct
import time
from rclpy.logging import get_logger


class Script:
    def __init__(self):
        self.id_mvt = 0

    def log_mvt(self, node):
        node.write_log(f"Mouvement ID: {self.id_mvt}")
        self.id_mvt += 1

    def run(self, node):
        node.write_log("Script PD is running...")

        # Banderole
        node.kalman(False)
        node.send_raw("VMAX 0.1")
        node.sleep(0.5)
        node.move_to(Position(1.22, 0.12, 2.03), seuil=0.05)
        node.wait_for_motion()
        node.stepper(STEPPER_struct(2))
        node.sleep(0.5)
        node.stepper(STEPPER_struct(1))
        node.add_score(20)

        node.send_raw("VMAX 0.5")
        node.relative_move_to(Position(0, 0.15, 0), seuil=0.05)
        node.wait_for_motion()

        node.stepper(STEPPER_struct(0))

        # Fin banderole
        node.kalman(True)

        # Deplacement vers boites
        node.send_raw("VMAX 0.5")
        node.servo(SERVO_struct(1, 10))
        node.servo(SERVO_struct(2, 180))
        node.sleep(1)
        self.log_mvt(node)
        node.move_to(Position(1.22, 0.6, 2.03))
        node.wait_for_motion()
        node.move_to(Position(0.8, 0.6, 2.03))
        node.wait_for_motion()

        # Ramassage des boites
        node.send_raw("VMAX 0.2")
        node.sleep(1)
        node.sleep(0.5)
        self.log_mvt(node)
        node.move_to(Position(0.8, 0.2, 2.03), seuil=0.05)
        node.wait_for_motion()
        self.log_mvt(node)
        node.sleep(1)
        node.sleep(1)
        node.sleep(1)
        node.move_to(Position(0.8, 0.9, 2.03), seuil=0.05)
        node.wait_for_motion()
        node.sleep(1)

        for _ in range(20):
            node.send_raw("VMAX 0.5")
            node.move_to(Position(0.6, 1.4, -0.95), seuil=0.05)
            node.wait_for_motion()
            node.sleep(0.5)

            node.send_raw("VMAX 0.5")
            node.move_to(Position(0.6, 0.4, -0.95), seuil=0.05)
            node.wait_for_motion()
            node.sleep(0.5)
