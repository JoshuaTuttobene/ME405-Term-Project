"""!
@file encoder_reader.py

Creates an Encoder class that contains code needed to get measurements from encoders.
The class within this file contains functions to zero and read encoder values.

@author Aaron Escamilla, Karen Morales De Leon, Joshua Tuttobene
@date   02/15/2024 Original program
@copyright (c) 2023 by Spluttflob and released under the GNU Public Licenes V3
"""
import pyb

class Encoder:
    """! 
    This class implements a encoder for an ME405 kit. 
    """

    def __init__ (self, pin_A, pin_B, timer):
        """! 
        Initializes the encoder reader by enabling the pins and the timer
        @param en_pin (There will be several parameters)
        """
        # Set up for each parameter. This will be called in other program
        self.pin_A = pin_A
        self.pin_B = pin_B
        self.timer = timer
        self.ch1 = timer.channel(1, pyb.Timer.ENC_AB, pin=pin_A)
        self.ch2 = timer.channel(2, pyb.Timer.ENC_AB, pin=pin_B)
        self.previous_reading = 0
        self.current_location = 0
        #print ("Initializing")

    def read (self):
        """!
        This method reads the current value of the encoder.
        Must be called frequently to maintain accuracy.
        """
        self.current_position = self.timer.counter()
        self.delta = self.current_position - self.previous_reading
        self.previous_reading = self.current_position
        
        # Underflow
        if self.delta >= (2**16)//2:
            self.delta -= 2**16
        
        # Overflow
        elif self.delta <= -(2**16)//2:
            self.delta += 2**16
            
        self.current_location += self.delta
        return self.current_location
        
    def zero (self):
        """!
        This method resets the encoder counter to 0. To function properly, begin reading
        encoder data soon after calling this function.
        """
        #print("Zeroed")
        self.timer.counter(0)
        self.current_location = 0
        self.previous_reading = 0
        self.delta = 0
        self.current_position = 0
        return self.timer.counter()
    
