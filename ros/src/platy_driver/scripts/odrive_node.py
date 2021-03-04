#!/usr/bin/env python3

"""
TODO:
    - config from params
    - config save
    - exception, error checking
    - tune controller
    - watchdog, failsafe
    - sensor data analyze/publish
    - odometry calculation
"""

import rospy
from geometry_msgs.msg import Twist

import time
import sys

from odrive_driver import ODriveDriver

left_speed = 0.0
right_speed = 0.0

def cmd_vel_callback(msg):
    global left_speed
    global right_speed
    
    left_speed  =   msg.linear.x - msg.angular.z
    right_speed = -(msg.linear.x + msg.angular.z)

def main():
    global left_speed
    global right_speed

    rospy.init_node("odrive_node")
    rospy.Subscriber("cmd_vel", Twist, cmd_vel_callback, queue_size=2)

    odrive = ODriveDriver()
    
    odrive.calibrate()
    odrive.engage()
    
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        odrive.set_velocity(left_speed, right_speed)
        odrive.update()

        rate.sleep()

    odrive.disengage()

if __name__ == '__main__':

    main()

    print("ODrive node exiting.")
