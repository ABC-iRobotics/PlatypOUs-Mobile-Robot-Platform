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

import odrive
from odrive.enums import *
import time
import sys

left_axis = None
right_axis = None

def cmd_vel_callback(cmd_vel):
    if(left_axis != None and right_axis != None):
        if(left_axis.current_state == 8 and right_axis.current_state == 8):
            left_speed = 2 * cmd_vel.linear.x - cmd_vel.angular.z
            right_speed = -(2 * cmd_vel.linear.x + cmd_vel.angular.z)
            
            left_axis.controller.input_vel = left_speed * 2
            right_axis.controller.input_vel = right_speed * 2
    
if __name__ == '__main__':
    rospy.init_node("odrive_node")
    rospy.Subscriber("/cmd_vel", Twist, cmd_vel_callback, queue_size=2)

    odrv0 = odrive.find_any()

    print("\nConnected to " + str(odrv0.serial_number))
    print("HW version:  " + str(odrv0.hw_version_major) + "." + str(odrv0.hw_version_minor) + " - " + str(odrv0.hw_version_variant))
    print("FW version:  " + str(odrv0.fw_version_major) + "." + str(odrv0.fw_version_minor) + "." + str(odrv0.fw_version_revision))
    print("API version: " + odrive.version.get_version_str())
    print("Voltage:     " + str(odrv0.vbus_voltage))
    
    left_axis = odrv0.axis0
    right_axis = odrv0.axis1

    odrv0.config.brake_resistance = 5
    odrv0.config.max_regen_current = 10
    odrv0.config.dc_max_negative_current = -10

    right_axis.motor.config.resistance_calib_max_voltage = 3
    right_axis.motor.config.calibration_current = 5
    right_axis.motor.config.pole_pairs = 5
    right_axis.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT
    right_axis.motor.config.current_lim = 7.0
    right_axis.motor.config.current_control_bandwidth = 50

    right_axis.encoder.config.cpr = 2048
    right_axis.encoder.config.mode = ENCODER_MODE_INCREMENTAL

    right_axis.controller.config.vel_limit = 10
    right_axis.controller.config.vel_gain = 0.5
    right_axis.controller.config.vel_integrator_gain = 0
    right_axis.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL

    right_axis.error = 0
    right_axis.motor.error = 0
    right_axis.encoder.error = 0
    right_axis.controller.error = 0

    left_axis.motor.config.resistance_calib_max_voltage = 3
    left_axis.motor.config.calibration_current = 5
    left_axis.motor.config.pole_pairs = 5
    left_axis.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT
    left_axis.motor.config.current_lim = 7.0
    left_axis.motor.config.current_control_bandwidth = 50

    left_axis.encoder.config.cpr = 2048
    left_axis.encoder.config.mode = ENCODER_MODE_INCREMENTAL

    left_axis.controller.config.vel_limit = 10
    left_axis.controller.config.vel_gain = 0.5
    left_axis.controller.config.vel_integrator_gain = 0
    left_axis.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL

    left_axis.error = 0
    left_axis.motor.error = 0
    left_axis.encoder.error = 0
    left_axis.controller.error = 0

    if(left_axis.motor.is_calibrated == False or left_axis.encoder.is_ready == False or right_axis.motor.is_calibrated == False or right_axis.encoder.is_ready == False):
        print("\nCalibrating...")

        left_axis.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        right_axis.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        while(left_axis.current_state != 1):
            time.sleep(0.2)
            sys.stdout.write('.')
            sys.stdout.flush()
        while(right_axis.current_state != 1):
            time.sleep(0.2)
            sys.stdout.write('.')
            sys.stdout.flush()
        print("")
    else:
        print("Already calibrated, skipping...")

    print("Left axis error code: ", hex(left_axis.error))
    print("Left motor error code: ", hex(left_axis.motor.error))
    print("Left encoder error code: ", hex(left_axis.encoder.error))
    print("Left controller error code: ", hex(left_axis.controller.error))
    print("Right axis error code: ", hex(right_axis.error))
    print("Right motor error code: ", hex(right_axis.motor.error))
    print("Right encoder error code: ", hex(right_axis.encoder.error))
    print("Right controller error code: ", hex(right_axis.controller.error))

    if not(left_axis.error == 0 and left_axis.motor.error == 0 and left_axis.encoder.error == 0 and left_axis.controller.error == 0 and right_axis.error == 0 and right_axis.motor.error == 0 and right_axis.encoder.error == 0 and right_axis.controller.error == 0):
        print("Error during calibration.")
    else:
        print("Calibration successful. Entering closed-loop control.")
    right_axis.controller.input_vel = 0
    right_axis.requested_state = 8
    left_axis.controller.input_vel = 0
    left_axis.requested_state = 8

    right_axis.config.enable_watchdog = True
    right_axis.config.watchdog_timeout = 0.2

    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        
        try:
            right_axis.watchdog_feed()
            
            if not(left_axis.error == 0 and left_axis.motor.error == 0 and left_axis.encoder.error == 0 and left_axis.controller.error == 0 and right_axis.error == 0 and right_axis.motor.error == 0 and right_axis.encoder.error == 0 and right_axis.controller.error == 0):            
                print("ODrive error.")
                print("Left axis error code: ", hex(left_axis.error))
                print("Left motor error code: ", hex(left_axis.motor.error))
                print("Left encoder error code: ", hex(left_axis.encoder.error))
                print("Left controller error code: ", hex(left_axis.controller.error))
                print("Right axis error code: ", hex(right_axis.error))
                print("Right motor error code: ", hex(right_axis.motor.error))
                print("Right encoder error code: ", hex(right_axis.encoder.error))
                print("Right controller error code: ", hex(right_axis.controller.error))
                right_axis.clear_errors()
                right_axis.requested_state = 8
        except:
            print("Exception.")
            
        rate.sleep()
    
    print("ODrive node exiting.")
    right_axis.requested_state = 1
    left_axis.requested_state = 1
