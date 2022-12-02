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
from nav_msgs.msg import Odometry
import numpy as np

import time
import sys

from odrive_driver import ODriveDriver

class ODriveNode:

    wheel_separation = 0.42
    wheel_radius = 0.05
    
    left_speed = 0.0
    right_speed = 0.0
    
    topic_timer = 0.0
    topic_timeout = 0.5
    is_timed_out = True
    
    frequency = 100.0
    
    def __init__(self):
        rospy.init_node("odrive")
        rospy.Subscriber("~cmd_vel", Twist, self.cmd_vel_callback, queue_size=1)
        
        odom_pub = rospy.Publisher("/odometry/wheel", Odometry, queue_size=1)
        odom_msg = Odometry()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"
        
        voltage_pub = rospy.Publisher("~status/voltage", Float64, queue_size=1)
        voltage_msg = Float64()
        
        #current_pub = rospy.Publisher("~status/current", Float64, queue_size=2)
        #current_msg = Float64()
        
        # ~ temp_l_pub = rospy.Publisher("~status/temperature_left", Float64, queue_size=2)
        # ~ temp_l_msg = Float64()
        
        # ~ temp_r_pub = rospy.Publisher("~status/temperature_right", Float64, queue_size=2)
        # ~ temp_r_msg = Float64()
        
        #status_pub = rospy.Publisher("~status/status", String, queue_size=2)
        #status_msg = String()
        
        #error_pub = rospy.Publisher("~status/errors", String, queue_size=2)
        #error_msg = String()

        odrive = ODriveDriver()
        
        rate = rospy.Rate(self.frequency)

        # f = open("odometry_data_" + str(time.time_ns()) + ".csv", "w")
        # f.write("t,R,L\n")

        prev_time = time.time_ns()
        
        rp = -odrive.get_encoder_right()
        lp = odrive.get_encoder_left()

        x = y = theta = v = omega = 0.0

        while not rospy.is_shutdown():
            try:
                odrive.update()

                while not rospy.is_shutdown():

                    if self.topic_timer < self.topic_timeout:
                        odrive.set_velocity(self.left_speed, self.right_speed)
                        self.is_timed_out = False
                    else:
                        odrive.set_velocity(0, 0)
                        self.is_timed_out = True

                    self.topic_timer += 1.0 / self.frequency

                    e_r = -odrive.get_encoder_right()
                    e_l = odrive.get_encoder_left()

                    delta_r = e_r - rp
                    delta_l = e_l - lp

                    rp = e_r
                    lp = e_l

                    dt = time.time_ns() - prev_time
                    hz = 1000000000 / dt
                    prev_time += dt

                    delta_t = dt / 1000000000
                    delta_r = delta_r * 0.00153398078788564123
                    delta_l = delta_l * 0.00153398078788564123

                    delta_d = self.wheel_radius * (delta_r + delta_l) / 2.0
                    delta_theta = self.wheel_radius * (delta_r - delta_l) / self.wheel_separation

                    v = delta_d / delta_t
                    omega = delta_theta / delta_t

                    x += delta_d * np.cos(theta + delta_theta / 2.0)
                    y += delta_d * np.sin(theta + delta_theta / 2.0)
                    theta += delta_theta


                    # v_r = odrive.get_velocity_right() * 2 * 3.14159265
                    # v_l = odrive.get_velocity_left() * 2 * 3.14159265

                    odom_msg.header.stamp = rospy.Time.now()
                    odom_msg.pose.pose.position.x = x
                    odom_msg.pose.pose.position.y = y
                    odom_msg.pose.pose.orientation.z = np.sin(theta / 2.0)
                    odom_msg.pose.pose.orientation.w = np.cos(theta / 2.0)
                    odom_msg.twist.twist.linear.x = v
                    odom_msg.twist.twist.angular.z = omega
                    odom_pub.publish(odom_msg)

                    voltage_msg.data = odrive.get_voltage()
                    voltage_pub.publish(voltage_msg)

                    #current_msg.data = odrive.get_current()
                    #current_pub.publish(current_msg)

                    # ~ temp_l_msg.data = odrive.get_temperature_left()
                    # ~ temp_l_pub.publish(temp_l_msg)

                    # ~ temp_r_msg.data = odrive.get_temperature_right()
                    # ~ temp_r_pub.publish(temp_r_msg)

                    #status_msg.data = odrive.get_status_string()
                    #if not self.is_timed_out:
                    #    status_msg.data += " (Active)"
                    #else:
                    #    status_msg.data += " (Timed out)"
                    #status_pub.publish(status_msg)

                    #error_msg.data = str(odrive.get_errors())
                    #error_pub.publish(error_msg)

                    rate.sleep()

                    #print("{:.2f} ms ({:.2f} hz)  e_r = {}  e_l = {}  v_r = {:.2f}  v_l = {:.2f}".format(dt / 1000000, hz, e_r, e_l, v_r, v_l))
                    # f.write("{},{},{}\n".format(ts, -e_r, e_l))


            except Exception as e:
                status_msg.data = "Connection error."
                status_pub.publish(status_msg)
                print(e)

        odrive.disengage()
        f.close()
    
    def cmd_vel_callback(self, msg):
        self.left_speed  = ((msg.linear.x - (msg.angular.z * self.wheel_separation / 2)) / self.wheel_radius) / (2 * 3.14159265)
        self.right_speed = ((msg.linear.x + (msg.angular.z * self.wheel_separation / 2)) / self.wheel_radius) / (2 * 3.14159265)
        self.topic_timer = 0.0

if __name__ == '__main__':
    odrvnode = ODriveNode()
    print("ODrive node exiting.")
