"""!
@file CL_PID.py
This file contains a class to perfrom closed loop proportional control.

@author Aaron Escamilla, Karen Morales De Leon, Joshua Tuttobene
@date   03/14/2024 Original program, based on example from above listed source
@copyright (c) 2023 by Spluttflob and released under the GNU Public Licenes V3
"""
class ClosedLoop_PID:
    """! 
    This class implements a closed loop PID controller.
    """
    def __init__ (self, Kp, Ki, Kd, setpoint,delta_t):
        """! 
        Initializes the Closed loop controller class. 
        @param Kp Kp for proportional control
        @param Ki Ki for integral control
        @param Kd Kd for derivative control
        @param setpoint Desired point to be reached by controller
        @param delta_t change in time to be used for integral and derivative calculations
        """
        self.kp = Kp
        self.ki = Ki
        self.kd = Kd
        self.setpoint = setpoint
        self.delta_t = delta_t
        self.integral = 0
        self.prev_error = setpoint
    
    def run(self,position):
        """! 
        This function runs the PID controller. It takes in the current position and
        compares to desired position to determine error and run necessary calculations
        before returning a pwm that should be sent to the motor.
        @param position the current position of whatever device is being controlled
        @returns returns a pwm value that should be sent to a motor
        """
        self.position = position
        error = (self.setpoint-self.position)
        P_gain = self.kp*(error)
        y = self.integral + (error*self.delta_t)
        I_gain = self.ki*y
        derivative = (error-self.prev_error) / self.delta_t   # put over delta t to actually make d controller
        D_gain = self.kd*derivative
        pwm = P_gain + I_gain + D_gain
        self.integral = y
        self.prev_error = error
        
        if pwm > 100:
            pwm = 100
        elif pwm < -100:
            pwm = -100
        
        
        return pwm
    
    def set_setpoint(self,setpoint):
        """! 
        Allows new setpoint to be entered without reinitializing class.
        @param setpoint Desired point to be reached by controller
        @returns prints setpoint value
        """
        self.setpoint = setpoint
        return print('setpoint:',self.setpoint)
    
    def set_gains(self,kp, ki, kd):
        """! 
        Allows new Kp to be entered without reinitializing class.
        @param Kp Kp for proportional control
        @param Ki Ki for integral control
        @param Kd Kd for derivative control
        @returns prints current gain values
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        return print('Kp:',self.kp, 'Ki:', self.ki, 'Kd:', self.kd)
    
if __name__ == "__main__":
    import motor_driver as MD
    import encoder_reader as ER
    import utime
    # Motor init 1
    enable_pin = pyb.Pin(pyb.Pin.board.PA10, pyb.Pin.OUT_PP)
    in1pin = pyb.Pin.cpu.B4
    in2pin = pyb.Pin.cpu.B5
    tim3 = pyb.Timer(3, freq=20000)
    motor = MD.MotorDriver(enable_pin, in1pin, in2pin, tim3)
    motor.enable()

    # Encoder init 1
    pin_A = pyb.Pin.cpu.C6
    pin_B = pyb.Pin.cpu.C7
    tim8 = pyb.Timer(8, prescaler = 0, period = 2**16-1)
    encoder = ER.Encoder(pin_A, pin_B, tim8)

    kp = 17#float(input("Enter a Kp value:"))  # input for Kp
    ki = 0#float(input("Enter a Ki value:"))  
    kd = 15#float(input("Enter a Kd value:"))
    sp = 144#float(input("Enter a setpoint:"))   # input for setpoint for run
    
    PID = ClosedLoop_PID(kp,ki,kd,sp,10/1000)
    encoder.zero()
    start = utime.ticks_ms()
    while True:
        try:
            utime.sleep_ms(10)
            pwm = PID.run(encoder.read())      # set return from controller as pwm for motor
            motor.set_duty_cycle(pwm)         # set new pwm
            print(utime.ticks_ms() - start,encoder.read())
            #if encoder.read() == 144:
                #break
        except KeyboardInterrupt:
            break
    motor.set_duty_cycle(0)
    print(encoder.read())
    

    
        

