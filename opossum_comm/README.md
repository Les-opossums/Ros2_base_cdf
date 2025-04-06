# OPOSSUM COMM

This package contains the nodes needed to communicate with the robot (currently only the Zynq), or simulate a communication.

First build the opossum_comm package (and the cdf_msgs) and then source the workspace:

```
colcon build --packages-select opossum_comm cdf_msgs
source install/setup.bash
```

Then you can launch the node and its parameters contained in the config folder:

```
ros2 launch opossum_comm comm.launch.py # By default, it will try to communicate with the Zynq
```

If you want to simulate the communication, you can run with specific parameter:

```
ros2 launch opossum_comm comm.launch.py simulation:=true
```

As for simulation, you can add as many robots as you want (only `main_robot` by default), by adding `robot_names:=main_robot,ennemi_robot,third_robot` for example. However, you have to check that configuration file is set for these robots.
