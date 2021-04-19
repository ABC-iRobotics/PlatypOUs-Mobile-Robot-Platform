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
        
        odom_msg = TwistWithCovarianceStamped()
        odom_msg.header.frame_id = "base_link"

        odrive = ODriveDriver()
        
        rate = rospy.Rate(10)
        
        while not rospy.is_shutdown():

            print(odrive.get_status_string())
            print(odrive.get_errors())
            print()
            
            odrive.make_ready()
            odrive.clear_errors()

            odrive.set_velocity(self.left_speed, self.right_speed)

            vels = odrive.get_velocity()

            odom_msg.header.stamp = rospy.Time.now()
            odom_msg.twist.twist.linear.x = (((vels[1] * 2 * 3.14159265) + (vels[0] * 2 * 3.14159265)) / 2.0) * self.wheel_radius
            odom_msg.twist.twist.angular.z = ((((vels[1] * 2 * 3.14159265) - (vels[0] * 2 * 3.14159265)) / 2.0) / (self.wheel_separation / 2)) * self.wheel_radius
            odom_pub.publish(odom_msg)
                
            rate.sleep()

        odrive.disengage()
    
    def cmd_vel_callback(self, msg):
        self.left_speed  = ((msg.linear.x - (msg.angular.z * self.wheel_separation / 2)) / self.wheel_radius) / (2 * 3.14159265)
        self.right_speed = ((msg.linear.x + (msg.angular.z * self.wheel_separation / 2)) / self.wheel_radius) / (2 * 3.14159265)

if __name__ == '__main__':
    odrvnode = ODriveNode()
    print("ODrive node exiting.")
