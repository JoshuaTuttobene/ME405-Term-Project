"""!
@file Servo_Driver.py
This file creates a driver to run a dc servo at the according time
to pull the trigger on the Nerf gun.

@author Aaron Escamilla, Karen Morales De Leon, Joshua Tuttobene
@date   03/09/2024 
"""

import pyb
import utime

class ServoDriver:
    """!
    The ServoDriver class implements a servo driver for the Nerf Gun Term Project
    """
    def __init__ (self, pwm, min_period, max_period, angle_range):
        """! 
        Creates a servo driver by initializing GPIO
        pins and ensuring the servo parameters specifications are met. 
        @param pwm: pulse width modulation set up with channel at corresponding frequency
        @param min_pw: The minimum pulse width range as specified by the servo
        @param max_pw: The maximum pulse width range as specified by the servo
        @param angle_range: The total allowable range the servo can turn
        """
        self.pwm = pwm
        self.min_pw = min_pw
        self.max_pw = max_pw
        self.angle_range = angle_range
        self.pulse_width_range = max_pw - min_pw
    
    def set_angle (self, angle):
        """! 
        Sets the desired angle to pull the trigger on the Nerf Gun.
        Equation below calculates the appropriate pulse width based on the given angle.
        @param angle: Necessary angle to fully activate the trigger
        """
        self.angle = angle
        self.pulse_width_eqn = int(((angle/self.angle_range)*self.pulse_width_range + self.min_pw)*1.75)
        self.pwm.pulse_width(self.pulse_width_eqn)
        print(f'Angle for {self.pulse_width_eqn} in microseconds')
 
# Test file to ServoDriver class that ensures it functions properly
if __name__ == "__main__":
    # Set up each parameter: pin, timer and channel
    servo_pin = pyb.Pin.cpu.B6
    
    # Set period and prescaler to give freq = 50Hz
    tim4 = pyb.Timer(4, prescaler = 79, period = 19999)
    ch1 = tim4.channel(1, pyb.Timer.PWM, pin=servo_pin)
    
    # Working cycle is from 500-2500microsecs, with 180 angle (from servo specs)
    min_pw = 500
    max_pw = 2500
    angle_range = 180
    
    # Create the servo driver
    servo = ServoDriver(ch1, min_pw, max_pw, angle_range)
    
    servo.set_angle(15)
    utime.sleep_ms(35)
    servo.set_angle(30)
    utime.sleep_ms(100)
    servo.set_angle(15)
    utime.sleep_ms(50)
