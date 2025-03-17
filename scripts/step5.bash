#!/bin/bash
source /opt/ros/humble/setup.bash
cd ~/uros_seminar_ws
source install/local_setup.bash
ros2 run rqt_plot rqt_plot topics /pico_sensor/forward_r /pico_sensor/forward_l /pico_sensor/right /pico_sensor/left
