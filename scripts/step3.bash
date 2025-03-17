#!/bin/bash
source /opt/ros/humble/setup.bash
ros2 service call /micro_ros_arduino_service std_srvs/srv/Trigger
