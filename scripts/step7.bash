#!/bin/bash
source /opt/ros/humble/setup.bash
cd ~/uros_seminar_ws
source install/local_setup.bash
ros2 launch pico_description display.launch.py
