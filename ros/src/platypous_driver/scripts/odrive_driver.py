#!/usr/bin/env python3

import odrive
from odrive.enums import *

import time
import sys

class ODriveDriver:
    
    odrv       = None
    left_axis  = None
    right_axis = None
    
    brake_resistance = 5
    max_regen_current = 10
    dc_max_negative_current = -10
    
    motor_resistance_calib_max_voltage = 3
    motor_calibration_current = 5
    motor_pole_pairs = 5
    motor_type = MOTOR_TYPE_HIGH_CURRENT
    motor_current_lim = 7.0
    motor_current_control_bandwidth = 50
    
    encoder_cpr = 2048
    encoder_mode = ENCODER_MODE_INCREMENTAL
    
    controller_vel_limit = 10
    controller_vel_gain = 0.5
    controller_vel_integrator_gain = 0
    controller_control_mode = CONTROL_MODE_VELOCITY_CONTROL
    
    left_vel_multiplier  =  2
    right_vel_multiplier = -2
    
    
    def __init__(self):
        self.update()
    
    
    def connect(self):
        if self.is_connected():
            return

        self.odrv = odrive.find_any()
        
        try:
            self.left_axis = self.odrv.axis1
            self.right_axis = self.odrv.axis0
        except:
            return

        self.odrv.config.enable_brake_resistor = True
        self.odrv.config.brake_resistance = self.brake_resistance
        self.odrv.config.max_regen_current = self.max_regen_current
        self.odrv.config.dc_max_negative_current = self.dc_max_negative_current

        self.right_axis.motor.config.resistance_calib_max_voltage = self.left_axis.motor.config.resistance_calib_max_voltage = self.motor_resistance_calib_max_voltage
        self.right_axis.motor.config.calibration_current = self.left_axis.motor.config.calibration_current = self.motor_calibration_current
        self.right_axis.motor.config.pole_pairs = self.left_axis.motor.config.pole_pairs = self.motor_pole_pairs
        self.right_axis.motor.config.motor_type = self.left_axis.motor.config.motor_type = self.motor_type
        self.right_axis.motor.config.current_lim = self.left_axis.motor.config.current_lim = self.motor_current_lim
        self.right_axis.motor.config.current_control_bandwidth = self.left_axis.motor.config.current_control_bandwidth = self.motor_current_control_bandwidth
                                                                    
        self.right_axis.encoder.config.cpr = self.left_axis.encoder.config.cpr = self.encoder_cpr
        self.right_axis.encoder.config.mode = self.left_axis.encoder.config.mode = self.encoder_mode
                                                                    
        self.right_axis.controller.config.vel_limit = self.left_axis.controller.config.vel_limit = self.controller_vel_limit
        self.right_axis.controller.config.vel_gain = self.left_axis.controller.config.vel_gain = self.controller_vel_gain
        self.right_axis.controller.config.vel_integrator_gain = self.left_axis.controller.config.vel_integrator_gain = self.controller_vel_integrator_gain
        self.right_axis.controller.config.control_mode = self.left_axis.controller.config.control_mode = self.controller_control_mode
                                                                    
        self.right_axis.error = self.left_axis.error = 0
        self.right_axis.motor.error = self.left_axis.motor.error = 0
        self.right_axis.encoder.error = self.left_axis.encoder.error = 0
        self.right_axis.controller.error = self.left_axis.controller.error = 0

    
    def calibrate(self):
        if (not self.is_connected()) or self.is_calibrating() or self.is_calibrated():
            return
        
        self.clear_errors()
        self.disengage()
        self.left_axis.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        self.right_axis.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE

        while(self.is_calibrating()):
            time.sleep(0.1)


    def engage(self):
        if (not self.is_calibrated()) or self.is_engaged():
            return
        
        self.clear_errors()
        
        self.left_axis.controller.input_vel = 0
        self.left_axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        
        self.right_axis.controller.input_vel = 0
        self.right_axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

        self.left_axis.config.enable_watchdog = True
        self.left_axis.config.watchdog_timeout = 0.25
        self.left_axis.watchdog_feed()
        
        self.right_axis.config.enable_watchdog = True
        self.right_axis.config.watchdog_timeout = 0.25
        self.right_axis.watchdog_feed()


    def disengage(self):
        if not self.is_connected():
            return
            
        self.left_axis.controller.input_vel = 0
        self.left_axis.config.enable_watchdog = False
        self.left_axis.config.watchdog_timeout = 0.0
        self.left_axis.requested_state = AXIS_STATE_IDLE
        
        self.right_axis.controller.input_vel = 0
        self.right_axis.config.enable_watchdog = False
        self.right_axis.config.watchdog_timeout = 0.0
        self.right_axis.requested_state = AXIS_STATE_IDLE


    def update(self):
        self.connect()
        self.calibrate()
        self.engage()


    def set_velocity(self, left_vel, right_vel):
        self.left_axis.controller.input_vel = left_vel * self.left_vel_multiplier
        self.right_axis.controller.input_vel = right_vel * self.right_vel_multiplier
        self.left_axis.watchdog_feed()
        self.right_axis.watchdog_feed()


    def get_velocity_left(self):
        return (self.left_axis.encoder.vel_estimate / self.left_vel_multiplier)


    def get_velocity_right(self):
        return (self.right_axis.encoder.vel_estimate / self.right_vel_multiplier)


    def get_encoder_left(self):
        return self.left_axis.encoder.shadow_count

    def get_encoder_right(self):
        return self.right_axis.encoder.shadow_count


    def get_voltage(self):
        return self.odrv.vbus_voltage


    def get_current(self):
        if not self.is_connected():
            return 0
        
        return self.odrv.ibus


    # ~ def get_temperature_left(self):
        # ~ if not self.is_connected():
            # ~ return 0
        
        # ~ return self.odrv.left_axis.fet_thermistor.temp


    # ~ def get_temperature_right(self):
        # ~ if not self.is_connected():
            # ~ return 0
        
        # ~ return self.odrv.right_axis.fet_thermistor.temperature



    def is_connected(self):
        try:
            return self.odrv and self.odrv.axis0 and self.odrv.axis1
        except:
            return False


    def is_error(self):
        if not self.is_connected():
            return False
        
        errors = self.get_errors()
        
        return errors[0] or errors[1] or errors[2] or errors[3] or errors[4] or errors[5] or errors[6] or errors[7] or errors[8]


    def is_calibrated(self):
        if not self.is_connected():
            return False
        
        return self.left_axis.motor.is_calibrated and self.left_axis.encoder.is_ready and self.right_axis.motor.is_calibrated and self.right_axis.encoder.is_ready


    def is_calibrating(self):
        if not self.is_connected():
            return False
        
        return (not (self.left_axis.current_state == AXIS_STATE_IDLE or self.left_axis.current_state == AXIS_STATE_CLOSED_LOOP_CONTROL)) or (not (self.right_axis.current_state == AXIS_STATE_IDLE or self.right_axis.current_state == AXIS_STATE_CLOSED_LOOP_CONTROL))


    def is_engaged(self):
        if not self.is_connected():
            return False
        
        return self.left_axis.current_state == AXIS_STATE_CLOSED_LOOP_CONTROL and self.right_axis.current_state == AXIS_STATE_CLOSED_LOOP_CONTROL


    def is_ready(self):
        return self.is_engaged() and not self.is_error()


    def get_errors(self):
        if not self.is_connected():
            return [0, 0, 0, 0, 0, 0, 0, 0, 0]
        
        return [self.odrv.error, self.left_axis.error, self.left_axis.motor.error, self.left_axis.encoder.error, self.left_axis.controller.error, self.right_axis.error, self.right_axis.motor.error, self.right_axis.encoder.error, self.right_axis.controller.error]


    def clear_errors(self):
        if not self.is_connected():
            return
        
        self.odrv.clear_errors()


    def get_status_string(self):
        if self.is_ready():
            return "Ready"
        elif self.is_engaged():
            return "Engaged"
        elif self.is_calibrated():
            return "Calibrated"
        elif self.is_calibrating():
            return "Calibrating"
        elif self.is_connected():
            return "Connected"
        else:
            return "Not connected"
