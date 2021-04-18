#!/bin/bash

source /root/ros_ws/devel/setup.bash

roslaunch platypous_launch full.launch &
sleep 5 && cd /root/web && node server.js &

wait
