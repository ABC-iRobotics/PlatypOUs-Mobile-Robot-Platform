#!/usr/bin/env python3

"""
TODO:
    - config from params
    - sensor data publish
    - diagnostics
"""

import rospy
from std_msgs.msg import Float64
from std_msgs.msg import String
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
        
        voltage_pub = rospy.Publisher("status/voltage", Float64, queue_size=2)
        voltage_msg = Float64()
        
        current_pub = rospy.Publisher("status/current", Float64, queue_size=2)
        current_msg = Float64()
        
        # ~ temp_l_pub = rospy.Publisher("status/temperature_left", Float64, queue_size=2)
        # ~ temp_l_msg = Float64()
        
        # ~ temp_r_pub = rospy.Publisher("status/temperature_right", Float64, queue_size=2)
        # ~ temp_r_msg = Float64()
        
        status_pub = rospy.Publisher("status/status", String, queue_size=2)
        status_msg = String()
        
        error_pub = rospy.Publisher("status/errors", String, queue_size=2)
        error_msg = String()

        odrive = ODriveDriver()
        
        rate = rospy.Rate(10)
        
        while not rospy.is_shutdown():
            
            try:
                odrive.make_ready()
                odrive.clear_errors()

                odrive.set_velocity(self.left_speed, self.right_speed)

                odom_msg.header.stamp = rospy.Time.now()
                odom_msg.twist.twist.linear.x = (((odrive.get_velocity_right() * 2 * 3.14159265) + (odrive.get_velocity_left() * 2 * 3.14159265)) / 2.0) * self.wheel_radius
                odom_msg.twist.twist.angular.z = ((((odrive.get_velocity_right() * 2 * 3.14159265) - (odrive.get_velocity_left() * 2 * 3.14159265)) / 2.0) / (self.wheel_separation / 2)) * self.wheel_radius
                odom_pub.publish(odom_msg)
                
                voltage_msg.data = odrive.get_voltage()
                voltage_pub.publish(voltage_msg)
                
                current_msg.data = odrive.get_current()
                current_pub.publish(current_msg)
                
                # ~ temp_l_msg.data = odrive.get_temperature_left()
                # ~ temp_l_pub.publish(temp_l_msg)

                # ~ temp_r_msg.data = odrive.get_temperature_right()
                # ~ temp_r_pub.publish(temp_r_msg)
                
                status_msg.data = odrive.get_status_string()
                status_pub.publish(status_msg)
                
                error_msg.data = str(odrive.get_errors())
                error_pub.publish(error_msg)
            
            except Exception as e:
                status_msg.data = "Connection error."
                status_pub.publish(status_msg)
                raise
            
            rate.sleep()

        odrive.disengage()
    
    def cmd_vel_callback(self, msg):
        self.left_speed  = ((msg.linear.x - (msg.angular.z * self.wheel_separation / 2)) / self.wheel_radius) / (2 * 3.14159265)
        self.right_speed = ((msg.linear.x + (msg.angular.z * self.wheel_separation / 2)) / self.wheel_radius) / (2 * 3.14159265)

if __name__ == '__main__':
    odrvnode = ODriveNode()
    print("ODrive node exiting.")
