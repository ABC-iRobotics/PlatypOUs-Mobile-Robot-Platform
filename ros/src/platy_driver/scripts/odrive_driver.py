#!/usr/bin/env python3

import odrive
from odrive.enums import *

class ODriveDriver:
    
    self.odrv       = None
    self.left_axis  = None
    self.right_axis = None
    
    def __init__(self):
        odrv = odrive.find_any()

        print("\nConnected to " + str(odrv.serial_number))
        print("HW version:  " + str(odrv.hw_version_major) + "." + str(odrv.hw_version_minor) + " - " + str(odrv.hw_version_variant))
        print("FW version:  " + str(odrv.fw_version_major) + "." + str(odrv.fw_version_minor) + "." + str(odrv.fw_version_revision))
        print("API version: " + odrive.version.get_version_str())
        print("Voltage:     " + str(odrv.vbus_voltage))
        
        left_axis = odrv.axis0
        self.right_axis = odrv.axis1

        odrv.config.brake_resistance = 5
        odrv.config.max_regen_current = 10
        odrv.config.dc_max_negative_current = -10

        self.right_axis.motor.config.resistance_calib_max_voltage = 3
        self.right_axis.motor.config.calibration_current = 5
        self.right_axis.motor.config.pole_pairs = 5
        self.right_axis.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT
        self.right_axis.motor.config.current_lim = 7.0
        self.right_axis.motor.config.current_control_bandwidth = 50

        self.right_axis.encoder.config.cpr = 2048
        self.right_axis.encoder.config.mode = ENCODER_MODE_INCREMENTAL

        self.right_axis.controller.config.vel_limit = 10
        self.right_axis.controller.config.vel_gain = 0.5
        self.right_axis.controller.config.vel_integrator_gain = 0
        self.right_axis.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL

        self.right_axis.error = 0
        self.right_axis.motor.error = 0
        self.right_axis.encoder.error = 0
        self.right_axis.controller.error = 0

        self.left_axis.motor.config.resistance_calib_max_voltage = 3
        self.left_axis.motor.config.calibration_current = 5
        self.left_axis.motor.config.pole_pairs = 5
        self.left_axis.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT
        self.left_axis.motor.config.current_lim = 7.0
        self.left_axis.motor.config.current_control_bandwidth = 50

        self.left_axis.encoder.config.cpr = 2048
        self.left_axis.encoder.config.mode = ENCODER_MODE_INCREMENTAL

        self.left_axis.controller.config.vel_limit = 10
        self.left_axis.controller.config.vel_gain = 0.5
        self.left_axis.controller.config.vel_integrator_gain = 0
        self.left_axis.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL

        self.left_axis.error = 0
        self.left_axis.motor.error = 0
        self.left_axis.encoder.error = 0
        self.left_axis.controller.error = 0
    
    def calibrate(self):
        if(self.left_axis.motor.is_calibrated == False or self.left_axis.encoder.is_ready == False or self.right_axis.motor.is_calibrated == False or self.right_axis.encoder.is_ready == False):
            print("\nCalibrating...")

            self.left_axis.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
            self.right_axis.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
            while(self.left_axis.current_state != 1):
                time.sleep(0.2)
                sys.stdout.write('.')
                sys.stdout.flush()
            while(self.right_axis.current_state != 1):
                time.sleep(0.2)
                sys.stdout.write('.')
                sys.stdout.flush()
            print("")
        else:
            print("Already calibrated, skipping...")

        print("Left axis error code: ", hex(self.left_axis.error))
        print("Left motor error code: ", hex(self.left_axis.motor.error))
        print("Left encoder error code: ", hex(self.left_axis.encoder.error))
        print("Left controller error code: ", hex(self.left_axis.controller.error))
        print("Right axis error code: ", hex(self.right_axis.error))
        print("Right motor error code: ", hex(self.right_axis.motor.error))
        print("Right encoder error code: ", hex(self.right_axis.encoder.error))
        print("Right controller error code: ", hex(self.right_axis.controller.error))

        if not(self.left_axis.error == 0 and self.left_axis.motor.error == 0 and self.left_axis.encoder.error == 0 and self.left_axis.controller.error == 0 and self.right_axis.error == 0 and self.right_axis.motor.error == 0 and self.right_axis.encoder.error == 0 and self.right_axis.controller.error == 0):
            print("Error during calibration.")
        else:
            print("Calibration successful. Entering closed-loop control.")

    def engage(self):
        self.right_axis.controller.input_vel = 0
        self.right_axis.requested_state = 8
        self.left_axis.controller.input_vel = 0
        self.left_axis.requested_state = 8

        self.right_axis.config.enable_watchdog = True
        self.right_axis.config.watchdog_timeout = 0.2
    
    def disengage(self):
        self.right_axis.requested_state = 1
        self.left_axis.requested_state = 1
    
    def set_velocity(self, left_vel, right_vel):
        self.left_axis.controller.input_vel = left_vel
        self.right_axis.controller.input_vel = right_vel

    def update(self):
        try:
            self.right_axis.watchdog_feed()
            
            if not(self.left_axis.error == 0 and self.left_axis.motor.error == 0 and self.left_axis.encoder.error == 0 and self.left_axis.controller.error == 0 and self.right_axis.error == 0 and self.right_axis.motor.error == 0 and self.right_axis.encoder.error == 0 and self.right_axis.controller.error == 0):            
                print("ODrive error.")
                print("Left axis error code: ", hex(self.left_axis.error))
                print("Left motor error code: ", hex(self.left_axis.motor.error))
                print("Left encoder error code: ", hex(self.left_axis.encoder.error))
                print("Left controller error code: ", hex(self.left_axis.controller.error))
                print("Right axis error code: ", hex(self.right_axis.error))
                print("Right motor error code: ", hex(self.right_axis.motor.error))
                print("Right encoder error code: ", hex(self.right_axis.encoder.error))
                print("Right controller error code: ", hex(self.right_axis.controller.error))
                self.right_axis.clear_errors()
                self.right_axis.requested_state = 8
        except:
            print("Exception.")
