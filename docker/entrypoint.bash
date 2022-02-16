#!/bin/bash

source /opt/ros/noetic/setup.bash
source ~/PlatypOUs-Mobile-Robot-Platform/ros/devel/setup.bash

export ROS_MASTER_URI=http://localhost:11311


if [ "$#" = 1 ] && [ "$1" = "build" ]; then
    echo "Building workspace."
    cd PlatypOUs-Mobile-Robot-Platform

    if [ ! -d "web/node_modules" ]; then
        cd web && npm ci && cd ..
    fi

    cd ros && catkin build

elif [ "$#" = 1 ] && [ "$1" = "start" ]; then
    echo "Starting."

    roslaunch platypous_launch main.launch &
    cd ~/PlatypOUs-Mobile-Robot-Platform/web && node server.js &

    wait

elif [ "$#" = 1 ] && [ "$1" = "bash" ]; then
    echo "Starting bash."

    bash

else
    echo "Wrong argument."
fi
