#!/bin/bash
source /opt/ros/humble/setup.bash
cd ~/uros_seminar_ws
source install/local_setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
