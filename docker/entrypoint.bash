#!/bin/bash

source /opt/ros/noetic/setup.bash

cd $1
exec ./platypous $2
