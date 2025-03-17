#!/bin/bash
cd ~
mkdir -p ~/uros_seminar_ws/src
cd ~/uros_seminar_ws/src
git clone -b humble https://github.com/micro-ROS/micro_ros_setup
sudo apt update && rosdep update
source /opt/ros/humble/setup.bash
cd ~/uros_seminar_ws
rosdep install --from-paths src --ignore-src -y
colcon build
source ./install/local_setup.bash
ros2 run micro_ros_setup create_agent_ws.sh
colcon build
source ./install/local_setup.bash

