#!/bin/bash
source /opt/ros/humble/setup.bash
cd ~/uros_seminar_ws
cd src
git clone https://github.com/rt-net/pico_ros
sudo apt update && rosdep update
cd ~/uros_seminar_ws
rosdep install --from-paths src --ignore-src -y
sudo apt install ros-humble-teleop-twist-keyboard
colcon build

