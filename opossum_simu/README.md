# To test and run interface

Update the submodule resources, and then use `colcon build` in the root of the workspace.

Then You can run the simualtion and all other nodes, in 2 different terminals (do not forget to source them using `source install/setup.bash`):

```
ros2 launch localization obs_all.launch.py
```

and

```
ros2 launch opossum_simu simu_launch.py
```
