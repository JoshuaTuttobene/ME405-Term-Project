"""!
@file motor_driver.py
Run real or simulated dynamic response tests and plot the results. This program
demonstrates a way to make a simple GUI with a plot in it. It uses Tkinter, an
old-fashioned and ugly but useful GUI library which is included in Python by
default.

This file is based loosely on an example found at
https://matplotlib.org/stable/gallery/user_interfaces/embedding_in_tk_sgskip.html

@author Aaron Escamilla, Karen Morales De Leon, Joshua Tuttobene
@date   02/05/2024 Original program, based on example from above listed source
@copyright (c) 2023 by Spluttflob and released under the GNU Public Licenes V3
"""

import micropython
import pyb
import utime

class MotorDriver:
    """! 
    This class implements a motor driver for an ME405 kit. 
    """

    def __init__ (self, en_pin, in1pin, in2pin, timer):
        """! 
        Creates a motor driver by initializing GPIO
        pins and turning off the motor for safety. 
        @param en_pin (There will be several parameters)
        """
        
        # Set up for each parameter. This will be called in other program
        self.en_pin = en_pin
        self.in1pin = in1pin
        self.in2pin = in2pin
        self.timer = timer
        self.ch1 = timer.channel(1, pyb.Timer.PWM, pin=in1pin)
        self.ch2 = timer.channel(2, pyb.Timer.PWM, pin=in2pin)
        self.en_pin.low()
        
        print ("Creating a motor driver")

    def set_duty_cycle (self,level):
        """!
        This method sets the duty cycle to be sent
        to the motor to the given level. Positive values
        cause torque in one direction, negative values
        in the opposite direction.
        @param level A signed integer holding the duty
               cycle of the voltage sent to the motor 
        """
        
        
#         while True:
#             try:
#                 level = int(level)
#                 break
#             except:
#                 print("type an integer from -100 to 100")
#                 level = 0
            
        # Clockwise
        if level > 0 and level <= 100:
            self.ch1.pulse_width_percent(level)
            self.ch2.pulse_width_percent(0)
        
        # Counter Clockwise
        elif level < 0 and level >= -100:
            self.ch1.pulse_width_percent(0)
            self.ch2.pulse_width_percent(-level)
        
        # Braking
        elif level == 0:
            self.ch1.pulse_width_percent(0)
            self.ch2.pulse_width_percent(0)
            
        # upper saturation
        elif level > 100:
            level = 100
            self.ch1.pulse_width_percent(level)
            self.ch2.pulse_width_percent(0)
            
        
        # lower saturation
        elif level < -100:
            level = 100
            self.ch1.pulse_width_percent(0)
            self.ch2.pulse_width_percent(level)
        
        # Anything else is 'invalid'
        else:
            print("type an integer from -100 to 100")
        #print (f"Setting duty cycle to {level}")
        return
            
    def enable(self):
        """!
        This method enables the motor. No parameters required.
        Sets the enable pin high.
        """
        # To enable the motor
        self.en_pin.high()
        return
        
    def disable(self):
        """!
        This method disables the motor. No parameters required.
        Sets the enable pin low.
        """
        # to disable the motor
        self.en_pin.low()
        return

if __name__ == '__main__':
    enable_pin = pyb.Pin(pyb.Pin.board.PA10, pyb.Pin.OUT_PP)
    in1pin = pyb.Pin.cpu.B4
    in2pin = pyb.Pin.cpu.B5
    tim3 = pyb.Timer(3, freq=20000)
    motor = MotorDriver(enable_pin, in1pin, in2pin, tim3)
    motor.enable()
    motor.set_duty_cycle(50)
    
