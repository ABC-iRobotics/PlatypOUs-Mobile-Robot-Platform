#!/bin/bash

source /root/ros_ws/devel/setup.bash

roslaunch platypous_launch basic.launch &
cd /root/web && node server.js &

wait
