#!/bin/bash

source /opt/ros/noetic/setup.bash
source /root/ros_ws/devel/setup.bash

exec roslaunch odrive_node odrive_node.launch
