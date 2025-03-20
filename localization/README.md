## Localization with Lidar Node

TODO: Cerate simualtion environment for fake lidar laser ranges, and test Greg code.
TODO: Simulate ennemies and show them on the simulator.
TODO: Do more test on the cpp to check every single condition.

If you wanna run simulation, go to your ros_ws:

```
source install/setup.bash
ros2 launch localization cpp_simu_launch.py
```

And if you want to launch Python node:

```
source install/setup.bash
ros2 launch localization py_simu_launch.py
```

For testing on goal, open new terminal and go to ros_ws:

```
source install/setup.bash
ros2 launch orchestrator_gui gui.launch.py
```

## Computation time
With 4 beacons only, 0.3ms of prcessing / message in cpp
With 4 beacons only, 1-4ms of prcessing / message in python
