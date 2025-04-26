"""Simulate Odom wheels."""

import numpy as np


class OmnidirectionalController:
    """Simulate the wheels and command for the robot."""

    def __init__(self, kp, ki, kd, vmax, amax, wheel_radius, robot_radius, dt):
        self.Kp = np.array(kp)
        self.Ki = np.array(ki)
        self.Kd = np.array(kd)
        self.vmax = vmax  # [vx, vy, omega]
        self.amax = amax  # [ax, ay, alpha]
        self.r = wheel_radius
        self.R = robot_radius
        self.dt = dt

        self.integral_error = np.zeros(3)
        self.prev_error = np.zeros(3)
        self.prev_vel = np.zeros(3)

        # Wheel angles for 3 omni wheels
        self.phi = [0, 2 * np.pi / 3, 4 * np.pi / 3]
        self.IK = np.array([[-np.sin(p), np.cos(p), self.R] for p in self.phi])

    def wrap_to_pi(self, angle):
        """Wrap to -pi and pi the angle."""
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def update_naive(self, current_pose, goal_pose):
        """Update the command using naive method."""
        x, y, theta = current_pose
        xg, yg, thetag = goal_pose

        # --- 1. Compute errors in world frame
        error = np.array([xg - x, yg - y, self.wrap_to_pi(thetag - theta)])

        # --- 2. PID Controller in world frame
        self.integral_error += error * self.dt
        derivative_error = (error - self.prev_error) / self.dt
        self.prev_error = error

        v_world = (
            self.Kp * error + self.Ki * self.integral_error + self.Kd * derivative_error
        )

        # --- 3. Rotate velocity to robot frame
        c, s = np.cos(theta), np.sin(theta)
        rot = np.array([[c, s], [-s, c]])
        v_xy_robot = rot @ v_world[:2]
        v_robot = np.array([*v_xy_robot, v_world[2]])

        # --- 4. Apply velocity limits
        for i in range(3):
            v_robot[i] = np.clip(v_robot[i], -self.vmax[i], self.vmax[i])

        # --- 5. Apply acceleration limits
        delta_v = v_robot - self.prev_vel
        for i in range(3):
            max_dv = self.amax[i] * self.dt
            delta_v[i] = np.clip(delta_v[i], -max_dv, max_dv)
        v_robot = self.prev_vel + delta_v
        self.prev_vel = v_robot

        # --- 6. Inverse Kinematics
        wheel_speeds = (1 / self.r) * self.IK @ v_robot

        return wheel_speeds, v_robot, error

    def update_scaled(self, current_pose, goal_pose):
        """Update the command using scaling."""
        x, y, theta = current_pose
        xg, yg, thetag = goal_pose

        # --- 1. Compute errors in world frame
        error = np.array([xg - x, yg - y, self.wrap_to_pi(thetag - theta)])

        # --- 2. Normalize errors
        # These define "how far" we are from the goal, scaled by max velocities
        norm_errors = np.abs(error / self.vmax)
        max_norm = np.max(norm_errors)
        scale = 1.0 if max_norm == 0 else 1.0 / max_norm

        # Apply this scale to slow all components proportionally
        scaled_error = error * scale

        # --- 3. PID controller on scaled error
        self.integral_error += scaled_error * self.dt
        derivative_error = (scaled_error - self.prev_error) / self.dt
        self.prev_error = scaled_error

        v_world = (
            self.Kp * scaled_error
            + self.Ki * self.integral_error
            + self.Kd * derivative_error
        )

        # --- 4. Rotate to robot frame
        c, s = np.cos(theta), np.sin(theta)
        rot = np.array([[c, s], [-s, c]])
        v_xy_robot = rot @ v_world[:2]
        v_robot = np.array([*v_xy_robot, v_world[2]])

        # --- 5. Velocity limits
        for i in range(3):
            v_robot[i] = np.clip(v_robot[i], -self.vmax[i], self.vmax[i])

        # --- 6. Acceleration limits
        delta_v = v_robot - self.prev_vel
        for i in range(3):
            max_dv = self.amax[i] * self.dt
            delta_v[i] = np.clip(delta_v[i], -max_dv, max_dv)
        v_robot = self.prev_vel + delta_v
        self.prev_vel = v_robot

        # --- 7. Inverse kinematics
        wheel_speeds = (1 / self.r) * self.IK @ v_robot

        return wheel_speeds, v_robot, error

    def update_custom_martin(self, current_pose, goal_pose):
        """Send command using Martin algo on C."""
        x, y, theta = current_pose
        xg, yg, thetag = goal_pose

        # --- 1. Compute errors in world frame
        error = np.array([xg - x, yg - y, self.wrap_to_pi(thetag - theta)])

        # --- 2. Normalize errors
        # These define "how far" we are from the goal, scaled by max velocities
        d_reach = np.linalg.norm(error[:2])
        c, s = np.cos(theta), np.sin(theta)
        dir = np.arctan2(error[1], error[0])
        speed_order_d = np.clip(
            np.sqrt(2 * self.amax[0] * 0.9 * d_reach), -self.vmax[0], self.vmax[0]
        )
        speed_order_vt = np.clip(
            np.sign(error[2])
            * np.sqrt(2 * self.vmax[2] * np.abs(error[2]) * 0.9)
            * np.exp(-d_reach),
            -self.vmax[2],
            self.vmax[2],
        )
        vx_world = speed_order_d * np.cos(dir)
        vy_world = speed_order_d * np.sin(dir)
        v_world = np.array([vx_world, vy_world])
        # --- 4. Rotate to robot frame
        rot = np.array([[c, s], [-s, c]])
        v_xy_robot = rot @ v_world[:2]
        v_robot = np.array([*v_xy_robot, speed_order_vt])

        # --- 5. Velocity limits
        self.prev_vel = v_robot

        # --- 7. Inverse kinematics
        wheel_speeds = (1 / self.r) * self.IK @ v_robot

        return wheel_speeds, v_robot, error
