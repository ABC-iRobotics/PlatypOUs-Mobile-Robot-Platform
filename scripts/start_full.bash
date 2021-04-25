#!/bin/bash

source /root/ros_ws/devel/setup.bash

roslaunch platypous_launch full.launch &
cd /root/web && node server.js &

wait
