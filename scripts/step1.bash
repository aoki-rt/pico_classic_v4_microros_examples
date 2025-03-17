#!/bin/bash
source /opt/ros/humble/setup.bash
ros2 topic pub --once /micro_ros_arduino_subscriber std_msgs/msg/Int32 "data: $1"
