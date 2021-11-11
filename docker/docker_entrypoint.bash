#!/bin/bash

source /opt/ros/noetic/setup.bash
source /root/ros_ws/devel/setup.bash

export ROS_MASTER_URI=http://localhost:11311

roslaunch platypous_launch main.launch &
cd /root/web && node server.js &

wait
