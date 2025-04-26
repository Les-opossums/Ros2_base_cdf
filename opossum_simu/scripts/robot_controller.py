#!/usr/bin/env python3
"""Node that controls the wheels and the position of the robot."""

import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32MultiArray
from opossum_simu.OmnidirectionalController import OmnidirectionalController
from rclpy.executors import ExternalShutdownException
import csv
import os


class OmniControllerSim(Node):
    """Simulate the wheel speed and position."""

    def __init__(self):
        super().__init__("omni_controller_sim")
        self.declare_parameter("dt", 0.005)
        self.declare_parameter("factor_acceleration", 20.0)
        self.dt = self.get_parameter("dt").get_parameter_value().double_value
        self.factor_acceleration = (
            self.get_parameter("factor_acceleration").get_parameter_value().double_value
        )

        vmax = 1
        odo_spacing = 0.115
        amax = 0.3
        atmax = 0.3
        wheel_radius = 0.03
        self.controller = OmnidirectionalController(
            kp=[1000.0, 1000.0, 2000.0],
            ki=[0.0, 0.0, 0.0],
            kd=[0.1, 0.1, 0.3],
            vmax=[vmax, vmax, 2 * vmax / odo_spacing],
            amax=[amax, amax, atmax],
            wheel_radius=wheel_radius,
            robot_radius=0.2,
            dt=self.dt,
        )

        self.actual_wheel_speeds = np.zeros(3)
        self.pose = np.array([0.0, 0.0, 0.0])  # [x, y, theta]
        self.goal = np.array([2.0, 1.0, 3 * np.pi / 2])

        self.trajectory = []

        self.publisher = self.create_publisher(Float32MultiArray, "robot_pose", 10)
        self.timer = self.create_timer(
            self.dt / self.factor_acceleration, self.control_loop
        )

    def control_loop(self):
        """Control the loop."""
        # --- 1. Controller output (ideal wheel commands)
        self.wheel_cmds, v_desired, error = self.controller.update_scaled(
            self.pose, self.goal
        )
        # self.wheel_cmds = self.limit_wheel_speeds_proportional(0.3, 0.3, self.dt)
        # --- 2. Simulate motor lag and noise
        alpha = 0.1  # lag factor (closer to 1 = slower response)
        noise_std = 0.1  # rad/s

        # Apply lag and add noise
        for i in range(3):
            noise = np.random.normal(0, noise_std)
            self.actual_wheel_speeds[i] = (
                alpha * self.actual_wheel_speeds[i]
                + (1 - alpha) * self.wheel_cmds[i]
                + noise
            )

        w1, w2, w3 = self.actual_wheel_speeds

        # --- 3. Forward Kinematics: compute robot velocity in robot frame
        r = self.controller.r
        R = self.controller.R
        phi = self.controller.phi

        FK_matrix = np.array(
            [
                [-np.sin(phi[0]), -np.sin(phi[1]), -np.sin(phi[2])],
                [np.cos(phi[0]), np.cos(phi[1]), np.cos(phi[2])],
                [1 / R, 1 / R, 1 / R],
            ]
        ) * (r / 3.0)

        v_robot = FK_matrix @ np.array([w1, w2, w3])  # [vx, vy, omega]

        # --- 4. Convert to world frame
        theta = self.pose[2]
        c, s = np.cos(theta), np.sin(theta)
        v_world = np.array(
            [c * v_robot[0] - s * v_robot[1], s * v_robot[0] + c * v_robot[1]]
        )

        # --- 5. Integrate pose
        self.pose[0] += v_world[0] * self.dt
        self.pose[1] += v_world[1] * self.dt
        self.pose[2] += v_robot[2] * self.dt

        # --- 6. Publish + Log
        msg = Float32MultiArray()
        msg.data = [*self.pose, *v_robot, *error, w1, w2, w3]
        self.publisher.publish(msg)

        self.trajectory.append(
            [
                self.get_clock().now().nanoseconds / 1e9,
                *self.pose,
                *v_robot,
                *error,
                w1,
                w2,
                w3,
            ]
        )

        # --- 7. Stop if close enough
        if np.linalg.norm(error[:2]) < 0.01 and abs(error[2]) < 0.02:
            self.save_and_exit()

    def save_and_exit(self):
        """Save the file and exit."""
        self.get_logger().info("Goal reached. Saving log...")
        file_path = os.path.expanduser("/tmp/robot_log.csv")
        with open(file_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(
                [
                    "time",
                    "x",
                    "y",
                    "theta",
                    "vx",
                    "vy",
                    "omega",
                    "ex",
                    "ey",
                    "etheta",
                    "w1",
                    "w2",
                    "w3",
                ]
            )
            writer.writerows(self.trajectory)
        self.get_logger().info(f"Saved to {file_path}")
        rclpy.shutdown()

    def limit_wheel_speeds_proportional(self, max_speed, max_acc, dt):
        """Limit the speed of the wheels, proportionnaly for each wheel."""
        # Step 1: Limit speed magnitude (preserve ratio)
        max_cmd = np.max(np.abs(self.wheel_cmds))
        if max_cmd > max_speed:
            self.wheel_cmds *= max_speed / max_cmd

        # Step 2: Compute desired change in wheel speeds
        delta = self.wheel_cmds - self.actual_wheel_speeds
        delta_norm = np.linalg.norm(delta)  # magnitude of the delta vector
        max_delta = max_acc * dt

        # Step 3: Limit acceleration proportionally
        if delta_norm > max_delta:
            delta *= max_delta / delta_norm

        wheel_limited = self.actual_wheel_speeds + delta
        return wheel_limited


def main(args=None):
    """Spin main loop."""
    rclpy.init(args=args)
    node = OmniControllerSim()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
