"""!
@file CL_Proportional_Control.py
This file contains a class to perfrom closed loop proportional control.

@author Aaron Escamilla, Karen Morales De Leon, Joshua Tuttobene
@date   02/22/2024 Original program, based on example from above listed source
@copyright (c) 2023 by Spluttflob and released under the GNU Public Licenes V3
"""
class ClosedLoop_P:
    """! 
    This class implements a closed loop proportional controller.
    """
    def __init__ (self, Kp,setpoint):
        """! 
        Initializes the Closed loop controller class. 
        @param Kp Kp for proportional control
        @param setpoint Desired point to be reached by controller
        """
        self.kp = Kp
        self.setpoint = setpoint
    
    def run(self,position):
        """! 
        This function runs the P controller. It takes in the current position and
        compares to desired position to determine error before returning a pwm that
        should be sent to the motor. Saturation is not performed here as it is done
        in the motor class.
        @param position the current position of whatever device is being controlled
        """
        self.position = position
        pwm = self.kp*(self.setpoint-self.position)
        return pwm
    
    def set_setpoint(self,setpoint):
        """! 
        Allows new setpoint to be entered without reinitializing class.
        @param setpoint Desired point to be reached by controller
        """
        self.setpoint = setpoint
        return print('setpoint:',self.setpoint)
    
    def set_kp(self,kp):
        """! 
        Allows new Kp to be entered without reinitializing class.
        @param Kp Kp for proportional control
        """
        self.kp = kp
        return print('Kp:',self.kp)