#!/bin/bash
source /opt/ros/humble/setup.bash
cd ~/uros_seminar_ws
source ./install/local_setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
