#!/bin/bash
cd ~
git clone -b humble-esp32s3-add-custom-msg https://github.com/rt-net/micro_ros_arduino
cp -r ~/micro_ros_arduino/src/esp32s3 ~/Arduino/libraries/micro_ros_arduino/src/esp32s3
cp -r ~/micro_ros_arduino/src/pico_msgs ~/Arduino/libraries/micro_ros_arduino/src/pico_msgs
