"""Plot the trajectory and all data of the robot."""

import matplotlib.pyplot as plt
import csv

file_path = "/tmp/robot_log.csv"

times = []
x, y, theta = [], [], []
vx, vy, omega = [], [], []
ex, ey, etheta = [], [], []
w1, w2, w3 = [], [], []

with open(file_path) as f:
    reader = csv.DictReader(f)
    for row in reader:
        times.append(float(row["time"]))
        x.append(float(row["x"]))
        y.append(float(row["y"]))
        theta.append(float(row["theta"]))
        vx.append(float(row["vx"]))
        vy.append(float(row["vy"]))
        omega.append(float(row["omega"]))
        ex.append(float(row["ex"]))
        ey.append(float(row["ey"]))
        etheta.append(float(row["etheta"]))
        w1.append(float(row["w1"]))
        w2.append(float(row["w2"]))
        w3.append(float(row["w3"]))

# plt.figure()
# plt.plot(x, y, label='Trajectory')
# plt.plot(x[0], y[0], 'go', label='Start')
# plt.plot(x[-1], y[-1], 'ro', label='End')
# plt.xlabel('x [m]')
# plt.ylabel('y [m]')
# plt.title('Robot Trajectory')
# plt.legend()
# plt.grid()

# plt.figure()
# plt.plot(times, theta, label='Trajectory')
# plt.xlabel('Time [s]')
# plt.ylabel('Angle [rad]')
# plt.title('Angle robot during Robot Trajectory')
# plt.legend()
# plt.grid()

# # plt.figure()
# # plt.plot(times, ex, label='Error X')
# # plt.plot(times, ey, label='Error Y')
# # plt.plot(times, etheta, label='Error θ')
# # plt.xlabel('Time [s]')
# # plt.ylabel('Error')
# # plt.title('Error Over Time')
# # plt.legend()
# # plt.grid()

# plt.figure()
# plt.plot(times, vx, label='Speed X')
# plt.plot(times, vy, label='Speed Y')
# plt.plot(times, omega, label='Speed θ')
# plt.xlabel('Time [s]')
# plt.ylabel('Speed Robot')
# plt.title('Speed Over Time')
# plt.legend()
# plt.grid()

# plt.figure()
# plt.plot(times, w1, label='Wheel 1')
# plt.plot(times, w2, label='Wheel 2')
# plt.plot(times, w3, label='Wheel 3')
# plt.xlabel('Time [s]')
# plt.ylabel('Wheel Velocities [rad/s]')
# plt.title('Wheel Speeds Over Time')
# plt.legend()
# plt.grid()

# plt.show()


fig, axs = plt.subplots(2, 2, figsize=(12, 8))  # 2 rows, 2 columns

# Subplot 1: Robot Trajectory
axs[0, 0].plot(x, y, label="Trajectory")
axs[0, 0].plot(x[0], y[0], "go", label="Start")
axs[0, 0].plot(x[-1], y[-1], "ro", label="End")
axs[0, 0].set_xlabel("x [m]")
axs[0, 0].set_ylabel("y [m]")
axs[0, 0].set_title("Robot Trajectory")
axs[0, 0].legend()
axs[0, 0].grid()

# Subplot 2: Angle over time
axs[0, 1].plot(times, theta, label="Trajectory")
axs[0, 1].set_xlabel("Time [s]")
axs[0, 1].set_ylabel("Angle [rad]")
axs[0, 1].set_title("Angle during Robot Trajectory")
axs[0, 1].legend()
axs[0, 1].grid()

# Subplot 3: Speed over time
axs[1, 0].plot(times, vx, label="Speed X")
axs[1, 0].plot(times, vy, label="Speed Y")
axs[1, 0].plot(times, omega, label="Speed θ")
axs[1, 0].set_xlabel("Time [s]")
axs[1, 0].set_ylabel("Speed Robot")
axs[1, 0].set_title("Speed Over Time")
axs[1, 0].legend()
axs[1, 0].grid()

# Subplot 4: Wheel Speeds
axs[1, 1].plot(times, w1, label="Wheel 1")
axs[1, 1].plot(times, w2, label="Wheel 2")
axs[1, 1].plot(times, w3, label="Wheel 3")
axs[1, 1].set_xlabel("Time [s]")
axs[1, 1].set_ylabel("Wheel Velocities [rad/s]")
axs[1, 1].set_title("Wheel Speeds Over Time")
axs[1, 1].legend()
axs[1, 1].grid()

plt.tight_layout()
plt.show()
