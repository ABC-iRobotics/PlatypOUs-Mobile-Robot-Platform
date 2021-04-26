#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from std_msgs.msg import String

import random
import time
import sys


class DummyMotorDriver:
    
    def __init__(self):
        rospy.init_node("dummy_motor_driver")
        
        voltage_pub = rospy.Publisher("status/voltage", Float64, queue_size=2)
        voltage_msg = Float64()
        voltage_msg.data = 24.0;
        voltage_delta = 0.1 * random.random();
        
        current_pub = rospy.Publisher("status/current", Float64, queue_size=2)
        current_msg = Float64()
        current_msg.data = 1.0;
        current_delta = 0.02 * random.random();
        
        temp_l_pub = rospy.Publisher("status/temperature_left", Float64, queue_size=2)
        temp_l_msg = Float64()
        
        temp_r_pub = rospy.Publisher("status/temperature_right", Float64, queue_size=2)
        temp_r_msg = Float64()
        
        status_pub = rospy.Publisher("status/status", String, queue_size=2)
        status_msg = String()
        
        error_pub = rospy.Publisher("status/errors", String, queue_size=2)
        error_msg = String()
        
        rate = rospy.Rate(1)
        
        while not rospy.is_shutdown():
            
            voltage_msg.data += voltage_delta
            if(voltage_msg.data > 25.2):
                voltage_delta = -0.1 * random.random()
            if(voltage_msg.data < 18.0):
                voltage_delta = 0.1 * random.random()
            voltage_pub.publish(voltage_msg)
            
            current_msg.data += current_delta
            if(current_msg.data > 3.0):
                current_delta = -0.02 * random.random()
            if(current_msg.data < 0.0):
                current_delta = 0.02 * random.random()
            current_pub.publish(current_msg)
            
            # ~ temp_l_msg.data = odrive.get_temperature_left()
            # ~ temp_l_pub.publish(temp_l_msg)

            # ~ temp_r_msg.data = odrive.get_temperature_right()
            # ~ temp_r_pub.publish(temp_r_msg)
            
            status = random.randint(0, 5)
            if status == 0:
                status_msg.data = "Ready"
            elif status == 1:
                status_msg.data = "Engaged"
            elif status == 2:
                status_msg.data = "Calibrated"
            elif status == 3:
                status_msg.data = "Calibrating"
            elif status == 4:
                status_msg.data = "Connected"
            else:
                status_msg.data = "Not connected"
            status_pub.publish(status_msg)
            
            error_msg.data = str([hex(random.randint(0, 262144)), hex(random.randint(0, 262144)), hex(random.randint(0, 262144)), hex(random.randint(0, 262144)), hex(random.randint(0, 262144)), hex(random.randint(0, 262144)), hex(random.randint(0, 262144)), hex(random.randint(0, 262144))])
            error_pub.publish(error_msg)
            
            rate.sleep()
    

if __name__ == '__main__':
    dmd = DummyMotorDriver()
    print("DummyMotorDriver node exiting.")
