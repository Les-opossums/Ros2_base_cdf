Base Ros pour le robot oppossum

### After automatic launch
A service launch ros at boot. You can find the scripts in the utils folder.
To read logs on the raspberry, you can use the following command:

```roslog```


### For the test of the beacon detectors, and for the robot GUI during game, you have to do the following:

```
cd ~/your_ros_cdf_ws/
```

To build the packages of the GUI and of the robot simulation:

```
colcon build --symlink-install --packages-select opossum_msgs
colcon build --symlink-install --packages-select localization
colcon build --symlink-install --packages-select orchestrator_gui
```

Then source your ROS environment if not already done, and the workspace:

```
source /opt/ros/humble/setup.bash
source install/setup.bash
```

You can now run the GUI by running:

```
ros2 launch orchestrator_gui gui.launch.py
```

You should see an horrible interface. Here is only decoration currently. Go to page 2 to see the movements of the robot.

Now open a nem terminal to launch the simulation of the robot and the lidar localisation. Dont forget to source again:
If needed you can add the `source /opt/ros/humble/setup.bash` at the end of your `.bashrc` to not be bothered everytime and not type the following command. It is located in your `~/`. Else:
```
source /opt/ros/humble/setup.bash
```

And source the workspace
```
source install/setup.bash
```

Now you can run the robot simulation:

```
ros2 launch localization simulation_beacons_launch.py
```

You should see after some time on the interface the robot teleport to a random initial place.

Now you can send him orders. Open a new terminal, do not forget to source again and then run the following command with the position you want:

```
ros2 action send_goal --feedback /main_robot/moveto opossum_msgs/action/MoveTo "{goal: {x: 0.457, y: 0.367, z: 1}}"
```

You should see the robot moving on the board. Not really functional, so it can plant, and axis arent set well so maybe 0 is at the wrong place. You can send other goals.

When you run another action and the first has not ended it plants --> to be debugged...

And the final thing is that if you are on the page 2 of the interface you can also send goals clicking on the map and the robot should follow.
