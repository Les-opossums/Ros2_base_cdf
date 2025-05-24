#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Action Sequencer Node."""


import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterEvent

from std_msgs.msg import String, Bool, Int32
from opossum_msgs.msg import RobotData, LidarLoc

from opossum_action_sequencer.utils import (
    Position,
    PUMP_struct,
    LED_struct,
    SERVO_struct,
    STEPPER_struct,
    VALVE_struct,
)

import threading
from threading import Event
import time
import os


class ActionManager(Node):
    """Action Manager Node."""

    def __init__(self):
        super().__init__("action_sequencer_node")
        self.get_logger().info("Action Manager Node started")
        self._init_parameters()
        self._init_publishers()
        self._init_subscribers()
        self.state_leash = False
        self.script_class = None
        self.ready = False
        self.is_started = False

        # Action Done
        self.is_robot_moving = False
        self.is_robot_arrived = False
        self.is_pump_top_on = False
        self.is_pump_bottom_on = False
        self.robot_pos = None
        self.motion_done = True
        self.motion_done_event = Event()
        self.motion_done_event.set()

    def _init_parameters(self) -> None:
        """Initialize the parameters of the node."""
        self.declare_parameters(
            namespace="",
            parameters=[
                ("pub_command", "command"),
                ("feedback_topic", "/main_robot/feedback_command"),
                ("pub_score", "/main_robot/score"),
                ("pub_au", "/main_robot/au"),
                ("pub_feedback", "/main_robot/feedback_command"),
                ("pub_position", "/main_robot/position_out"),
                ("pub_velocity", "/main_robot/asserv/vel"),
            ],
        )

    def _init_publishers(self):
        """Initialize the publishers of the node."""
        self.pub_command = self.create_publisher(
            String,
            "/main_robot/command",
            10
        )

        self.pub_score = self.create_publisher(
            Int32,
            "/main_robot/score",
            10
        )

        self.pub_au = self.create_publisher(
            Bool,
            "/main_robot/au",
            10
        )

    def _init_subscribers(self):
        """Initialize the subscribers of the node."""
        self.subscription = self.create_subscription(
            ParameterEvent,
            "/parameter_events",
            self.parameter_event_callback,
            10
        )

        self.pub_feedback = self.create_subscription(
            String,
            "/main_robot/feedback_command",
            self.feedback_callback,
            10
        )

        self.robot_data_sub = self.create_subscription(
            RobotData,
            "/main_robot/robot_data",
            self.robot_data_callback,
            1
        )

        self.lidar_loc_sub = self.create_subscription(
            LidarLoc,
            "/main_robot/position_out",
            self.lidar_loc_callback,
            1
        )

    def parameter_event_callback(self, event):
        """Handle the parameters event."""
        # Parcours des paramètres modifiés
        # self.get_logger().info(f"Parameter event received: {event}")
        for changed in event.changed_parameters:
            if changed.name == "script_number":
                # Affiche la nouvelle valeur du paramètre script_number
                self.get_logger().info(
                    f"Choix du script : {changed.value.integer_value}"
                )
                # Import script
                if changed.value.integer_value == 0:
                    pass
                elif changed.value.integer_value == 1:
                    from opossum_action_sequencer.match.script1 import Script

                    self.get_logger().info("Script 1")
                elif changed.value.integer_value == 2:
                    from opossum_action_sequencer.match.script2 import Script

                    self.get_logger().info("Script 2")

                elif changed.value.integer_value == 3:
                    from opossum_action_sequencer.match.script3 import Script

                    self.get_logger().info("Script 3")

                elif changed.value.integer_value == 4:
                    from opossum_action_sequencer.match.script4 import Script

                    self.get_logger().info("Script 4")

                elif changed.value.integer_value == 5:
                    from opossum_action_sequencer.match.script5 import Script

                    self.get_logger().info("Script 5")

                elif changed.value.integer_value == 6:
                    from opossum_action_sequencer.match.script6 import Script

                    self.get_logger().info("Script 6")

                else:
                    from opossum_action_sequencer.match.script1 import Script

                    self.get_logger().info("Default script")

                if changed.value.integer_value != 0:
                    self.ready = True
                    self.script_class = Script
                else:
                    pass

            elif changed.name == "debug_mode":
                # Affiche la nouvelle valeur du paramètre debug_mode
                if changed.value.bool_value:
                    self.get_logger().info("Debug mode enabled")
                    from opossum_action_sequencer.match.script_init import Script as ScriptInit
                    self.script_init_class = ScriptInit
                    self.run_script_init()
                else:
                    self.get_logger().info("Debug mode disabled")

    def feedback_callback(self, msg):
        """Receive the data from Zynq."""
        # self.get_logger().info(f"Feedback received: {msg.data}")
        if msg.data.startswith("LEASH"):
            self.state_leash = True
            if msg.data.startswith("LEASH"):
                if self.ready:
                    self.is_started = True
                    self.get_logger().info("Leash activated")
                    self.script_instance = self.script_class()
                    thread = threading.Thread(
                        target=self.script_instance.run, args=(self,)
                    )
                    thread.start()

        elif msg.data.startswith("AU"):
            # self.get_logger().info(f"AU_test : {msg.data[-1]}")
            if msg.data[-1] == "1":
                self.get_logger().info("AU activated")
                self.pub_au.publish(Bool(data=True))
                if self.is_started:
                    self.get_logger().warn("Stopping script")
                    self.script_instance.stop()
            elif msg.data[-1] == "2":
                # self.get_logger().info("AU activated")
                self.pub_au.publish(Bool(data=True))
            else:
                self.get_logger().info("AU deactivated")
                self.pub_au.publish(Bool(data=False))

        elif msg.data.startswith("YELLOWSWITCH"):
            self.synchro_lidar()

        elif msg.data.startswith("BLUESWITCH"):
            self.get_logger().info("Reload Ros")
            os.system('systemctl --user restart launch.service')

    #def robot_data_callback(self, msg: RobotData):
    #    """Receive the Robot Datas from Zynq."""
    #    # self.get_logger().info(f"Robot data received: {msg}")
    #    self.robot_pos = Position(x=msg.x, y=msg.y, t=msg.theta)
    #    self.robot_speed = Position(x=msg.vlin, y=msg.vdir, t=msg.vt)
    #    if not self.motion_done:
    #        self.wait_for_arrival()
    #        self.wait_for_stop()
    #        if not self.is_robot_moving and self.is_robot_arrived:
    #            # self.get_logger().info("Robot stopped")
    #            self.motion_done_event.set()
    #            self.motion_done = True
#
    #        elif not self.is_robot_moving and not self.is_robot_arrived:
    #            # self.get_logger().info("Robot stopped but not arrived")
    #            # self.synchro_lidar()
    #            # self.move_to(self.pos_obj)
    #            pass

    def robot_data_callback(self, msg: RobotData):
        """Receive the Robot Data from Zynq."""
        self.robot_pos = Position(x=msg.x, y=msg.y, t=msg.theta)
        self.robot_speed = Position(x=msg.vlin, y=msg.vdir, t=msg.vt)

        if not self.motion_done:
            # Update motion state
            self.update_arrival_status()
            self.update_motion_status()

            # Final check
            if self.is_robot_arrived and not self.is_robot_moving:
                self.get_logger().info("Motion complete. Signaling.")
                self.motion_done = True
                self.motion_done_event.set()

    def run_script_init(self):
        """Run the script initialization."""
        assert self.script_init_class is not None
        self.script_init_instance = self.script_init_class()
        thread = threading.Thread(
            target=self.script_init_instance.run, args=(self,)
        )
        thread.start()

    def lidar_loc_callback(self, msg: LidarLoc):
        """Receive Lidar location."""
        self.lidar_pos = Position(
            x=msg.robot_position.x,
            y=msg.robot_position.y,
            t=msg.robot_position.z
        )

    def move_to(self, pos: Position, seuil=0.1):
        """Compute the move_to action."""
        self.seuil = seuil
        self.get_logger().info(f"Moving to : {pos.x} {pos.y} {pos.t}")
        self.motion_done = False
        self.is_robot_moving = True
        self.is_robot_arrived = False
        time.sleep(0.1)
        self.pub_command.publish(String(data=f"MOVE {pos.x} {pos.y} {pos.t}"))
        self.pos_obj = pos
        self.motion_done_event.clear()  # Block the wait
        # self.get_logger().info("Robot moving...")

    def relative_move_to(self, delta: Position, seuil=0.1):
        """Compute the relative move_to action."""
        self.seuil = seuil
        pos = Position(
            x=self.robot_pos.x + delta.x,
            y=self.robot_pos.y + delta.y,
            t=self.robot_pos.t + delta.t
        )
        self.get_logger().info(f"Moving to : {pos.x} {pos.y} {pos.t}")
        self.motion_done = False
        self.is_robot_moving = True
        self.is_robot_arrived = False
        time.sleep(0.1)
        self.pub_command.publish(String(data=f"MOVE {pos.x} {pos.y} {pos.t}"))
        self.pos_obj = pos
        self.motion_done_event.clear()  # Block the wait
        # self.get_logger().info("Robot moving...")

    # def wait_for_arrival(self):
    #     """Compute the wait_for_arrival action."""
    #     time.sleep(0.2)
    #     delta_x = abs(self.pos_obj.x - self.robot_pos.x)
    #     delta_y = abs(self.pos_obj.y - self.robot_pos.y)
    #     delta_t = abs(self.pos_obj.t - self.robot_pos.t)
    #     # Log des angles
    #     # self.get_logger().info(
    #     #     f"Robot position: {self.robot_pos.x} "
    #     #     f"{self.robot_pos.y} {self.robot_pos.t}"
    #     # )
    #     # self.get_logger().info(
    #     #     f"Object position: {self.pos_obj.x} "
    #     #     f"{self.pos_obj.y} {self.pos_obj.t}"
    #     # )
    #     # self.get_logger().info(
    #     #     f"Delta x: {delta_x}, Delta y: {delta_y}, Delta t: {delta_t}"
    #     # )
# 
    #     if delta_x < self.seuil and delta_y < self.seuil:
    #         if delta_t < self.seuil:
    #             self.get_logger().info("Robot arrived")
    #             self.is_robot_arrived = True
# 
    # def wait_for_stop(self):
    #     """Compute the wait_for_stop action."""
    #     speed_lin = abs(self.robot_speed.x)
    #     speed_t = abs(self.robot_speed.t)
    #     if speed_lin < 0.0001 and speed_t < 0.0001:
    #         # self.get_logger().info("Robot stopped")
    #         self.is_robot_moving = False


    def update_arrival_status(self):
        delta_x = abs(self.pos_obj.x - self.robot_pos.x)
        delta_y = abs(self.pos_obj.y - self.robot_pos.y)
        delta_t = abs(self.pos_obj.t - self.robot_pos.t)
        if delta_x < self.seuil and delta_y < self.seuil and delta_t < self.seuil:
            if not self.is_robot_arrived:
                self.get_logger().info("Robot has arrived.")
            self.is_robot_arrived = True

    def update_motion_status(self):
        speed_lin = abs(self.robot_speed.x)
        speed_t = abs(self.robot_speed.t)
        if speed_lin < 0.0001 and speed_t < 0.0001:
            if self.is_robot_moving:
                self.get_logger().info("Robot has stopped.")
            self.is_robot_moving = False

    def wait_for_motion(self):
        """Compute the wait_for_motion action."""
        self.get_logger().info("Waiting for robot to stop...")
        self.motion_done_event.wait()
        self.get_logger().info("Motion done")

    def servo(self, servo: SERVO_struct):
        """Compute the servo action."""
        self.pub_command.publish(String(data=f"SERVO {servo.servo_id} "
                                             f"{servo.angle}"
                                        )
                                 )

        self.get_logger().info(f"SERVO : SERVO {servo.servo_id} {servo.angle}")
        time.sleep(0.1)

    def pump(self, pump: PUMP_struct):
        """Compute the pump action."""
        self.pub_command.publish(String(data=f"PUMP {pump.pump_id} "
                                             f"{pump.enable}"
                                        )
                                 )

        self.get_logger().info(f"PUMP : PUMP {pump.pump_id} {pump.enable}")
        time.sleep(0.1)

    def led(self, led: LED_struct):
        """Compute the led action."""
        self.pub_command.publish(String(data=f"LED {led.red} "
                                             f"{led.green} {led.blue}")
                                 )

    def stepper(self, stepper: STEPPER_struct):
        """Compute the stepper action."""
        self.pub_command.publish(
            String(data=f"STEPPER1  {stepper.mode}")
        )
        self.get_logger().info(f"STEPPER : STEPPER1 {stepper.mode}")
        time.sleep(0.1)

    def valve(self, valve: VALVE_struct):
        """Compute the valve action."""
        self.pub_command.publish(String(data=f"VALVE {valve.valve_id} 1"))

        self.get_logger().info(f"VALVE {valve.valve_id} 1")
        time.sleep(0.1)

    def write_log(self, message):
        """Write logs."""
        self.get_logger().info(f"{message}")

    def send_raw(self, raw_command):
        """Send raw commands."""
        self.get_logger().info(f"Sending raw command: {raw_command}")
        self.pub_command.publish(String(data=raw_command))
        time.sleep(0.1)

    def kalman(self, kalman: bool):
        """Compute the kalman action."""
        if kalman:
            self.pub_command.publish(String(data="ENKALMAN 1"))
            self.get_logger().info("KALMAN ON")
        else:
            self.pub_command.publish(String(data="ENKALMAN 0"))
            self.get_logger().info("KALMAN OFF")
        time.sleep(0.1)

    def synchro_lidar(self):
        """Synchronize odom with lidar."""
        if self.robot_pos is None:
            return
        command = (
            f"SYNCHROLIDAR {self.lidar_pos.x} "
            f"{self.lidar_pos.y} {self.lidar_pos.t}"
        )
        self.get_logger().info(f"Synchro : {command}")
        self.pub_command.publish(String(data=f"{command}"))

    def add_score(self, score):
        """Update the score."""
        self.pub_score.publish(Int32(data=score))
        time.sleep(0.1)


def main(args=None):
    """Run main loop."""
    rclpy.init(args=args)
    action_manager_node = ActionManager()
    rclpy.spin(action_manager_node)
    action_manager_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
