#!/usr/bin/env python3

"""
TODO:
    - config from params
    - sensor data publish
    - diagnostics
"""

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistWithCovarianceStamped

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
        
        odom_pub = rospy.Publisher("odom", TwistWithCovarianceStamped, queue_size=2)

        odrive = ODriveDriver()
        
        rate = rospy.Rate(10)
        
        while not rospy.is_shutdown():
            if not odrive.is_connected():
                print("Connecting to ODrive.")
                odrive.connect(timeout=5)
            elif not odrive.is_calibrated():
                print("Calibrating ODrive.")
                odrive.calibrate()
            elif not odrive.is_engaged():
                print("Engaging ODrive.")
                odrive.engage()
            else:
                odrive.set_velocity(self.left_speed, self.right_speed)
                lv, rv = odrive.get_velocity()
                if not (lv == None or rv == None):
                    odom_msg = TwistWithCovarianceStamped()
                    odom_msg.header.stamp = rospy.Time.now()
                    odom_msg.header.frame_id = "base_link"
                    odom_msg.twist.twist.linear.x = (((rv * 2 * 3.14159265) + (lv * 2 * 3.14159265)) / 2.0) * self.wheel_radius
                    odom_msg.twist.twist.angular.z = ((((rv * 2 * 3.14159265) - (lv * 2 * 3.14159265)) / 2.0) / (self.wheel_separation / 2)) * self.wheel_radius
                    odom_pub.publish(odom_msg)
            
            if not odrive.is_ok():
                le, re = odrive.get_errors(clear=True)
                print("ODrive error:")
                print(le)
                print(re)
                print()
                
            rate.sleep()

        odrive.disengage()
    
    def cmd_vel_callback(self, msg):
        self.left_speed  = ((msg.linear.x - (msg.angular.z * self.wheel_separation / 2)) / self.wheel_radius) / (2 * 3.14159265)
        self.right_speed = ((msg.linear.x + (msg.angular.z * self.wheel_separation / 2)) / self.wheel_radius) / (2 * 3.14159265)

if __name__ == '__main__':
    odrvnode = ODriveNode()
    print("ODrive node exiting.")
