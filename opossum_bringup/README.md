# Opossum bringup

In this packages are located all the main launchers to run the robot.

The current launchers are the IHM, simulation, dev_gui, communication, localisation.

To run launchers, first build all the workspace, and then source the workspace:

```
colcon build
source install/setup.bash
```

## IHM launch
To run the IHM, run:

```
ros2 launch opossum_bringup ihm.launch.py
```

## Simulation launch
To run full simulation (except the IHM node):

```
ros2 launch opossum_bringup bringup_simu.launch.py
```

The previous command internally launch the follwing: Communication, simulation, localisation and dev_gui.

And equivalent would be:

```
ros2 launch opossum_dev_gui dev_gui.launch.py
ros2 launch opossum_comm comm.launch.py simulation:=true # Add parameter to specify it is simulation (not by default)
ros2 launch opossum_localisation localisation.launch.py simulation:=true # Add parameter to specify it is simulation (not by default)
ros2 launch opossum_simu simu.launch.py
```

You can also specify more than one robot in simulation, however you will have to check all the parameters are well defined in each configuration files:

```
ros2 launch opossum_bringup bringup_simu.launch.py robot_names:=main_robot,ennemi_robot,other_robot # By default, there is only the main_robot
```
