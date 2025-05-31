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
        node.write_log("Script 3 is running...")
        node.end_zone = Position(2.7, 1.7, 1.6)
        node.middle = Position(2.3, 1.5, 1.6)

        node.send_raw("AMAX 0.7")
        node.sleep(0.1)

        # Banderole
        node.kalman(False)
        node.send_raw("VMAX 0.1")
        node.sleep(0.5)
        # node.move_to(Position(1.22, 0.12, 2.03), seuil=0.05)
        node.move_to(Position(1.8, 0.1, 2.03))
        node.wait_for_motion()
        node.stepper(STEPPER_struct(3))
        node.sleep(1.5)
        node.stepper(STEPPER_struct(1))
        node.add_score(20)

        node.send_raw("VMAX 0.6")
        # node.relative_move_to(Position(0, 0.15, 0), seuil=0.05)
        # node.wait_for_motion()

        node.stepper(STEPPER_struct(1))

        # Fin banderole
        node.kalman(True)
        node.sleep(1)

        # Poussette
        node.send_raw("VMAX 0.6")
        node.sleep(0.1)
        node.move_to(Position(1.8, 0.5, 2.03))
        node.servo(SERVO_struct(1, 10))
        node.servo(SERVO_struct(2, 180))
        node.stepper(STEPPER_struct(1))
        node.wait_for_motion()

        node.move_to(Position(2.22, 0.5, 2.03))
        node.wait_for_motion()

        node.move_to(Position(2.22, 0.2, 2.03))
        node.wait_for_motion()
        node.add_score(4)

        ### Ramassage des boites
        ###---------------------

        # Deplacement vers boites en baissant les pinces et
        node.move_to(Position(2.4, 1.24, 2.03))
        node.wait_for_motion()

        node.move_to(Position(1.9, 1.24, 2.03))
        node.wait_for_motion()

        # Ramassage des boites
        node.relative_move_to(Position(0, -0.25, 0))
        node.wait_for_motion()

        node.send_raw("VMAX 0.6")
        node.sleep(0.1)
        node.move_to(Position(1.8, 0.35, 2.03))
        node.wait_for_motion()
        node.add_score(4)

        # Deplacement dans la zone adverse
        node.send_raw("VMAX 0.8")
        node.sleep(0.1)
        node.move_to(Position(1.8, 1.22, 0.57))
        node.wait_for_motion()
        node.move_to(Position(0.6, 1.25, 0.57))
        node.wait_for_motion()

        # Ramassage des boites adverses
        node.send_raw("VMAX 0.3")
        node.sleep(0.1)
        node.pump(PUMP_struct(1, 1))
        node.move_to(Position(0.35, 1.3, 0.57))
        node.wait_for_motion()
        node.kalman(False)
        node.relative_move_to(Position(-0.3, 0.05, 0))
        node.wait_for_motion()

        # Baisse le stepper
        node.sleep(0.1)
        node.stepper(STEPPER_struct(2))
        node.pump(PUMP_struct(2, 1))
        node.pump(PUMP_struct(3, 1))
        node.pump(PUMP_struct(4, 1))
        node.sleep(1.5)

        # Decalage des boites
        node.relative_move_to(Position(0, -0.45, 0))
        node.wait_for_motion()

        # Lache
        node.pump(PUMP_struct(2, 0))
        node.pump(PUMP_struct(3, 0))
        node.pump(PUMP_struct(4, 0)) 
        node.add_score(4)

        # Stop la pompe
        node.pump(PUMP_struct(1, 0))
        node.valve(VALVE_struct(1))
        node.valve(VALVE_struct(2))
        node.valve(VALVE_struct(3))
        node.valve(VALVE_struct(4))
        node.stepper(STEPPER_struct(1))

        # Retour dans la zone de depart
        node.kalman(True)
        node.sleep(0.5)
        node.send_raw("AMAX 1.5")
        node.sleep(0.1)

        # Retour a la base
        node.send_raw("VMAX 0.8")
        node.move_to(Position(0.7, 1.1, 0.0))
        node.wait_for_motion()
        node.send_raw("VMAX 1.5")
        node.move_to(Position(2.3, 1.1, 0.0))
        node.wait_for_motion()
        node.send_raw("VMAX 1.5")
        node.move_to(Position(2.3, 0.4, 1.6))
        node.wait_for_motion()
        node.send_raw("VMAX 1.5")
        node.move_to(Position(2.3, 1.4, 1.6))
        node.wait_for_motion()
        node.send_raw("VMAX 1.5")
        node.move_to(Position(2.3, 0.4, 1.6))
        node.wait_for_motion()
        node.send_raw("VMAX 1.5")
        node.move_to(Position(2.3, 1.4, 1.6))
        node.wait_for_motion()
        node.send_raw("VMAX 1.5")
        node.move_to(Position(2.3, 0.4, 1.6))
        node.wait_for_motion()
        node.send_raw("VMAX 1.5")
        node.move_to(Position(2.3, 1.4, 1.6))
        node.wait_for_motion()
        node.send_raw("VMAX 1.0")
        node.move_to(Position(2.7, 1.7, 1.6))
        node.wait_for_motion()
        node.in_end_zone = True
        node.add_score(10)
        node.is_ended = True
