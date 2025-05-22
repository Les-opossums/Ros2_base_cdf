#!/bin/bash

while ! xset q &>/dev/null; do
    sleep 1
    echo "Waiting for X server to start..."
done

source install/setup.bash
ros2 launch opossum_bringup debug.launch.py
