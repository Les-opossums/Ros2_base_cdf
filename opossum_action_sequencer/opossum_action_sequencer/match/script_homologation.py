#!/usr/bin/env python3
# -*- coding: utf-8 -*-


from opossum_action_sequencer.scripts.action_sequencer_node import ActionManager
from opossum_action_sequencer.action_manager import Position, VACCUMGRIPPER_struct
from rclpy.logging import get_logger


class Script:
    def __init__(self):
        self.id_mvt = 0

    def log_mvt(self, node):
        node.write_log(f"Mouvement ID: {self.id_mvt}")
        self.id_mvt += 1

    def run(self, node):
        node.write_log("Script PD is running...")
        node.end_zone = Position(0.5, 1.0, 0.0)
 
        node.send_raw("VMAX 0.8")
        node.move_to(Position(0.5, 1., 3.14))
        node.wait_for_motion()
 
        node.begin_centering()
        node.sleep(0.2)
        node.center_front_stack()

        # node.move_to(Position(0.42, 1., 3.14))
        # node.wait_for_motion()
        # node.sleep(0.5)
 
        node.vaccumgripper(VACCUMGRIPPER_struct(1, 1, 2))
        node.vaccumgripper(VACCUMGRIPPER_struct(0, 1, 2))
        node.sleep(2.5)
 
        node.move_to(Position(0.5, 0.5, 0.))
        node.wait_for_motion()
        node.sleep(0.5)
 
        node.vaccumgripper(VACCUMGRIPPER_struct(0, 2, 2))
        node.vaccumgripper(VACCUMGRIPPER_struct(1, 2, 2))
        node.sleep(0.5)

        #-----------------------------------------------#

        node.move_to(Position(0.5, 1.5, 3.14))
        node.wait_for_motion()
 
        node.begin_centering()
        node.sleep(0.2)
        node.center_front_stack()

        # node.move_to(Position(0.42, 1., 3.14))
        # node.wait_for_motion()
        # node.sleep(0.5)
 
        node.vaccumgripper(VACCUMGRIPPER_struct(0, 1, 2))
        node.vaccumgripper(VACCUMGRIPPER_struct(1, 1, 2))
        node.sleep(2.5)
 
        node.move_to(Position(0.5, 1., 0.))
        node.wait_for_motion()
        node.sleep(0.5)
 
        node.vaccumgripper(VACCUMGRIPPER_struct(0, 2, 2))
        node.vaccumgripper(VACCUMGRIPPER_struct(1, 2, 2))



