#!/bin/bash

while ! xset q &>/dev/null; do
    sleep 1
    echo "Waiting for X server to start..."
done

export ROS_DOMAIN_ID=69
source install/setup.bash
ros2 launch opossum_bringup debug.launch.py
