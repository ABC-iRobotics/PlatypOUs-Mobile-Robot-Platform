#!/usr/bin/env python3

"""
TODO:
    - config from params
    - tune controller
    - sensor data/odometry publish
"""

import rospy
from geometry_msgs.msg import Twist

import time
import sys

from odrive_driver import ODriveDriver

class ODriveNode:

    wheel_separation = 0.42
    wheel_radius = 0.05
    
    left_speed = 0.0
    right_speed = 0.0
    
    def __init__(self):
        rospy.init_node("odrive")
        rospy.Subscriber("cmd_vel", Twist, self.cmd_vel_callback, queue_size=2)

        odrive = ODriveDriver()
        
        rate = rospy.Rate(10)
        
        while not rospy.is_shutdown():
            if not odrive.is_connected():
                print("Connecting to ODrive.")
                odrive.connect(timeout=1)
            elif not odrive.is_calibrated():
                print("Calibrating ODrive.")
                odrive.calibrate()
            elif not odrive.is_engaged():
                print("Engaging ODrive.")
                odrive.engage()
            else
                odrive.set_velocity(self.left_speed, self.right_speed)
                lv, rv = odrive.get_velocity()
                print(str(lv) + "  " + str(rv))
            
            if not odrive.is_ok():
                odrive.get_errors(clear=True)
                
            rate.sleep()

        odrive.disengage()
    
    def cmd_vel_callback(self, msg):
        self.left_speed  = ((msg.linear.x - (msg.angular.z * self.wheel_separation / 2)) / self.wheel_radius) / (2 * 3.14159265)
        self.right_speed = ((msg.linear.x + (msg.angular.z * self.wheel_separation / 2)) / self.wheel_radius) / (2 * 3.14159265)

if __name__ == '__main__':
    odrvnode = ODriveNode()
    print("ODrive node exiting.")
