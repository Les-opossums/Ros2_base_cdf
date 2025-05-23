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
    def __init__(self):
        self._stop_event = threading.Event()

    def run(self, node):
        while not self._stop_event.is_set():
            node.write_log('Script 4 is running...')
            node.stepper(STEPPER_struct(0))
            node.send_raw('PUMP 1 1')
            time.sleep(1)
            node.send_raw('PUMP 1 0')
            time.sleep(1)
            node.move_to(Position(1.25, 0.23, -0.41))
            node.send_raw('FREE')
            # Deplacement vers boites
            node.send_raw('VMAX 0.5')
            time.sleep(1)
            node.move_to(Position(1.1, 0.6, -1.12))
            node.wait_for_motion()

            # Ramassage des boites
            node.send_raw('VMAX 0.2')
            time.sleep(1)
            node.pump(PUMP_struct(1, 1))
            # node.pump(PUMP_struct(3, 1))
            # node.pump(PUMP_struct(4, 1))
            time.sleep(0.5)
            node.move_to(Position(1.1, 0.85, -1.12), seuil=0.05)
            node.wait_for_motion()
            node.move_to(Position(1.1, 1., -1.12), seuil=0.05)
            node.wait_for_motion()
            time.sleep(1)
            # node.move_to(Position(1.1, 0.9, -0.6), seuil=0.02)
            # node.wait_for_motion()
            # node.move_to(Position(1.1, 0.9, -1.6), seuil=0.02)
            # node.wait_for_motion()
            # node.move_to(Position(1.16, 0.87, -1.12))
            # node.wait_for_motion()
            # time.sleep(1)
            # node.move_to(Position(1.04, 0.87, -1.12))
            # node.wait_for_motion()

            # Poussette vers zone
            node.move_to(Position(0.35, 1.7, -0.95))
            node.wait_for_motion()

            # CONSTRUCTION DOUBLE ETAGE

            # Montée planche
            node.stepper(STEPPER_struct(2))
            time.sleep(2)
            node.pump(PUMP_struct(2, 1))
            node.stepper(STEPPER_struct(1))
            time.sleep(2)

            node.kalman(False)

            # node.move_to(Position(0.35, 1.65, -0.95), seuil=0.05)
            # Recule et monte les boites
            node.relative_move_to(Position(0, -0.05, 0), seuil=0.01)
            node.wait_for_motion()

            node.pump(PUMP_struct(1, 0))
            node.valve(VALVE_struct(2))

            node.relative_move_to(Position(0, -0.1, 0), seuil=0.01)
            node.wait_for_motion()

            node.servo(SERVO_struct(2, 180))
            node.servo(SERVO_struct(1, 20))

            # Reavance pour construction
            node.relative_move_to(Position(0, 0.1, 0), seuil=0.01)
            node.wait_for_motion()

            # Construction gauche
            # node.move_to(Position(0.35, 1.65, -1.7), seuil=0.05)
            node.relative_move_to(Position(0, 0, -0.75), seuil=0.01)
            node.wait_for_motion()

            # node.move_to(Position(0.27, 1.7, -1.7), seuil=0.05)
            # node.relative_move_to(Position(-0.08, 0.05, 0), seuil=0.01)
            node.relative_move_to(Position(-0.0, 0.05, 0), seuil=0.01)
            node.wait_for_motion()

            node.servo(SERVO_struct(2, 20))

            node.relative_move_to(Position(0.0, -0.05, 0), seuil=0.01)
            node.wait_for_motion()
            node.relative_move_to(Position(0, 0, 0.75), seuil=0.01)
            node.wait_for_motion()

            # Construction droite
            node.relative_move_to(Position(0, 0, 0.75), seuil=0.01)
            node.wait_for_motion()
            node.relative_move_to(Position(0.0, 0.05, 0), seuil=0.01)
            node.wait_for_motion()

            node.servo(SERVO_struct(1, 180))
            node.relative_move_to(Position(-0.0, 0.05, 0), seuil=0.01)
            node.wait_for_motion()
            node.relative_move_to(Position(0, 0, -0.75), seuil=0.01)
            node.wait_for_motion()

            node.valve(VALVE_struct(3))

            time.sleep(1)
            node.kalman(True)
            time.sleep(1)
            """
            # Ramassage planche
            node.pump(PUMP_struct(2, 1))
            node.stepper(STEPPER_struct(2))
            time.sleep(2)
            node.stepper(STEPPER_struct(1))
            time.sleep(1)

            # Eteint pompe bas et recule
            node.pump(PUMP_struct(1, 0))
            node.move_to(Position(0.35, 1.55, -0.95), seuil=0.05)
            node.wait_for_motion()

            # Monte les boites
            node.send_raw('SERVO 1 20')
            node.send_raw('SERVO 2 180')
            time.sleep(1)

            # Avance et pose les objets
            node.move_to(Position(0.35, 1.7, -0.95), seuil=0.05)
            node.wait_for_motion()
            node.pump(PUMP_struct(3, 0))
            node.pump(PUMP_struct(4, 0))
            time.sleep(1)
            node.pump(PUMP_struct(2, 0))
            time.sleep(2)
            node.add_score(10)
            """
            node.send_raw('VMAX 0.5')
            time.sleep(1)
            node.move_to(Position(0.35, 1.55, -0.95))
            node.send_raw('SERVO 1 180')
            node.send_raw('SERVO 2 20')
            node.wait_for_motion()
            node.move_to(Position(1.05, 0.37, -2.6))
            node.wait_for_motion()

            break

    def stop(self):
        self._stop_event.set()
