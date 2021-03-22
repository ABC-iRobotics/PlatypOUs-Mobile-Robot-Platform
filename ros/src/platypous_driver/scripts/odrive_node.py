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
    
    left_speed = 0.0
    right_speed = 0.0
    
    def __init__(self):
        rospy.init_node("odrive")
        rospy.Subscriber("cmd_vel", Twist, self.cmd_vel_callback, queue_size=2)

        odrive = ODriveDriver()
        
        while not odrive.connect():
            time.sleep(0.1)
        
        odrive.calibrate()
        odrive.engage()
        
        rate = rospy.Rate(10)
        
        while not rospy.is_shutdown():
            odrive.set_velocity(left_speed, right_speed)
            odrive.update()
            odrive.get_vel()
            
            rate.sleep()

        odrive.disengage()
    
    def cmd_vel_callback(self, msg):
        self.left_speed  =  msg.linear.x - msg.angular.z
        self.right_speed =  msg.linear.x + msg.angular.z

if __name__ == '__main__':
    odrvnode = ODriveNode()
    print("ODrive node exiting.")
