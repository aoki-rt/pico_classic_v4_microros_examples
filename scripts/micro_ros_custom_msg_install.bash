#!/bin/bash
cd ~/uros_seminar_ws/src
git clone https://github.com/rt-net/pico_msgs.git
cd ~/uros_seminar_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -y
colcon build
source install/local_setup.bash
