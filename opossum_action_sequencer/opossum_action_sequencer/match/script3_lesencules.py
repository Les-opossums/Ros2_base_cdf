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

        # Banderole
        node.kalman(False)
        node.send_raw("VMAX 0.1")
        time.sleep(0.5)
        node.move_to(Position(1.22, 0.12, 2.03), seuil=0.05)
        node.wait_for_motion()
        node.stepper(STEPPER_struct(2))
        time.sleep(0.5)
        node.stepper(STEPPER_struct(1))
        node.add_score(20)

        node.send_raw("VMAX 0.5")
        node.relative_move_to(Position(0, 0.15, 0), seuil=0.05)
        node.wait_for_motion()

        node.stepper(STEPPER_struct(0))

        # Fin banderole
        node.kalman(True)

        # Deplacement vers boites
        time.sleep(1)
        self.log_mvt(node)
        node.move_to(Position(1.1, 0.6, -1.12))
        node.wait_for_motion()

        # Ramassage des boites
        node.send_raw("VMAX 0.2")
        time.sleep(0.5)
        node.pump(PUMP_struct(1, 1))
        time.sleep(0.5)
        self.log_mvt(node)
        node.move_to(Position(1.1, 0.85, -1.12), seuil=0.05)
        node.wait_for_motion()
        self.log_mvt(node)
        node.move_to(Position(1.1, 1.0, -1.12), seuil=0.05)
        node.wait_for_motion()
        time.sleep(0.5)

        # Poussette vers zone
        self.log_mvt(node)
        node.move_to(Position(0.35, 1.3, -0.95))
        node.wait_for_motion()

        self.log_mvt(node)
        node.move_to(Position(0.35, 1.7, -0.95))
        node.wait_for_motion()

        # CONSTRUCTION DOUBLE ETAGE
        node.send_raw("VMAX 0.2")

        # Montée planche
        node.write_log("Montée planche")
        node.write_log("---------------------------")
        node.send_raw("FREE")
        node.stepper(STEPPER_struct(2))
        time.sleep(2)
        node.pump(PUMP_struct(2, 1))
        node.pump(PUMP_struct(3, 1))
        node.pump(PUMP_struct(4, 1))
        node.stepper(STEPPER_struct(1))
        time.sleep(0.5)

        node.kalman(False)
        time.sleep(1)

        # Recule et monte les boites
        node.write_log("Recule et monte les boites")
        node.write_log("---------------------------")
        node.pump(PUMP_struct(1, 0))
        node.valve(VALVE_struct(2))
        time.sleep(0.5)

        self.log_mvt(node)
        node.relative_move_to(Position(0, -0.1, 0), seuil=0.02)
        node.wait_for_motion()

        # Boite a la moitie pour replacer planche
        node.write_log("Boite a la moitie pour replacer planche")
        node.write_log("---------------------------")
        node.servo(SERVO_struct(1, 95))
        node.servo(SERVO_struct(2, 95))

        node.relative_move_to(Position(0, 0.08, 0), seuil=0.02)
        node.wait_for_motion()
        node.relative_move_to(Position(0, -0.08, 0), seuil=0.02)
        node.wait_for_motion()

        node.servo(SERVO_struct(2, 180))
        node.servo(SERVO_struct(1, 10))

        # Reavance pour construction
        node.write_log("Reavance pour construction")
        node.write_log("---------------------------")
        self.log_mvt(node)

        # Construction gauche
        node.write_log("Construction gauche")
        node.write_log("---------------------------")
        time.sleep(1)
        self.log_mvt(node)
        node.relative_move_to(Position(0, 0, -0.75), seuil=0.02)
        node.wait_for_motion()

        time.sleep(1)
        self.log_mvt(node)
        node.relative_move_to(Position(0, 0.05, 0), seuil=0.02)
        node.wait_for_motion()
        time.sleep(1)

        node.servo(SERVO_struct(2, 10))

        # On se remet droit
        node.write_log("On se remet droit")
        node.write_log("---------------------------")
        time.sleep(1)
        self.log_mvt(node)
        node.relative_move_to(Position(0.0, -0.05, 0), seuil=0.02)
        node.wait_for_motion()
        time.sleep(1)
        self.log_mvt(node)
        # node.relative_move_to(Position(0, 0, 0.75), seuil=0.02)
        # node.wait_for_motion()

        # Construction droite
        node.write_log("Construction droite")
        node.write_log("---------------------------")
        self.log_mvt(node)
        node.relative_move_to(Position(0, 0, 1.5), seuil=0.02)
        node.wait_for_motion()
        time.sleep(1)
        self.log_mvt(node)
        node.relative_move_to(Position(0.0, 0.05, 0), seuil=0.02)
        node.wait_for_motion()
        time.sleep(1)

        time.sleep(1)
        node.servo(SERVO_struct(1, 180))

        # On se remet droit
        node.write_log("On se remet droit")
        node.write_log("---------------------------")
        time.sleep(1)
        time.sleep(1)
        self.log_mvt(node)
        node.relative_move_to(Position(0.0, -0.05, 0), seuil=0.02)
        node.wait_for_motion()
        time.sleep(1)
        self.log_mvt(node)
        time.sleep(1)
        node.relative_move_to(Position(0, 0, -0.75), seuil=0.02)
        node.wait_for_motion()

        # Lacher la planche
        node.write_log("Lacher la planche")
        node.write_log("---------------------------")
        time.sleep(1)
        node.relative_move_to(Position(0, 0.08, 0), seuil=0.05)
        node.wait_for_motion()
        node.pump(PUMP_struct(2, 0))
        node.pump(PUMP_struct(3, 0))
        node.pump(PUMP_struct(4, 0))
        node.valve(VALVE_struct(1))
        node.valve(VALVE_struct(3))
        node.valve(VALVE_struct(4))

        # Construction finie
        node.add_score(12)
        node.write_log("Construction finie")
        node.write_log("---------------------------")
        time.sleep(1)
        node.kalman(True)
        time.sleep(1)

        # Recule
        node.send_raw("VMAX 0.5")
        time.sleep(1)
        self.log_mvt(node)
        node.move_to(Position(0.35, 1.55, -0.95))
        node.send_raw("SERVO 1 180")
        node.send_raw("SERVO 2 10")
        node.wait_for_motion()
        time.sleep(1)

        # POUSSE LES BOITES DE LA SCENE
        # On se place en face
        self.log_mvt(node)
        node.move_to(Position(0.82, 1.45, -0.95))
        node.wait_for_motion()

        time.sleep(1)
        node.kalman(False)
        time.sleep(1)

        # On s'avance
        node.send_raw("VMAX 0.2")
        node.pump(PUMP_struct(1, 1))
        # node.move_to(Position(0.75, 1.6, -0.95), seuil=0.05)
        node.relative_move_to(Position(0, 0.20, 0), seuil=0.05)
        node.wait_for_motion()

        # Baisse la planche
        node.stepper(STEPPER_struct(2))
        time.sleep(2)
        node.pump(PUMP_struct(2, 1))
        node.pump(PUMP_struct(3, 1))
        node.pump(PUMP_struct(4, 1))

        # On décale
        node.relative_move_to(Position(-0.4, 0, 0), seuil=0.05)
        node.wait_for_motion()

        # On lache les boites et la planche
        node.pump(PUMP_struct(1, 0))
        node.valve(VALVE_struct(2))
        node.pump(PUMP_struct(2, 0))
        node.pump(PUMP_struct(3, 0))
        node.pump(PUMP_struct(4, 0))
        node.valve(VALVE_struct(1))
        node.valve(VALVE_struct(3))
        node.valve(VALVE_struct(4))
        node.stepper(STEPPER_struct(1))
        node.add_score(4)
        time.sleep(0.5)

        # On recule
        node.relative_move_to(Position(0, -0.3, 0), seuil=0.05)
        node.wait_for_motion()

        time.sleep(1)
        node.kalman(True)
        time.sleep(1)

        node.send_raw("VMAX 0.5")

        # Retour à la maison
        self.log_mvt(node)
        node.move_to(Position(1.05, 0.37, -2.6))
        node.wait_for_motion()
        node.add_score(10)
