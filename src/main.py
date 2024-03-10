# """!
# @file basic_tasks.py
#     This file contains a demonstration program that runs some tasks for motor control, an
#     inter-task shared variable, and a queue. The tasks don't really @b do
#     anything; the example just shows how these elements are created and run.
# 
# @author Aaron Escamilla, Karen Morales De Leon, Joshua Tuttobene
# @date   2024-Feb-28 JRR Created from the remains of previous example
# @copyright (c) 2015-2021 by JR Ridgely and released under the GNU
#     Public License, Version 2. 
# """
# 
# import gc
# import pyb
# import cotask
# import task_share
# 
# import utime
# import cqueue
# 
# 
# def task1_fun(shares):
#     """!
#     Task 1 puts things into a share and a queue. Task 1 is for the first motor we use.
#     It uses the motor class, encoder class,
#     and the closed loop proportional controller class
#     @param shares A list holding the share and queue used by this task
#     """
#     init = utime.ticks_ms()
#     while True:
#         if time.full() == False:
#             pwm = CL.run(encoder.read())      # set return from controller as pwm for motor
#             time.put(utime.ticks_ms()-init)   # put time into queue
#             pos.put(encoder.read())           # put position into queue
#             motor.set_duty_cycle(pwm)         # set new pwm
#         else:
#             motor.disable()
#             break
#         yield
#     return
# 
# def task2_fun(shares):
#     """!
#     Task 2 also puts thins into a share and a queue. Task 2 is for the secondary motor we use.
#     It uses the motor class, encoder class,
#     and the closed loop proportional controller class
#     @param shares A tuple of a share and queue from which this task gets data
#     """
#     init_2 = utime.ticks_ms()
#     while True:
#         if time_2.full() == False:
#             pwm = CL_2.run(encoder_2.read())      # set return from controller as pwm for motor
#             time_2.put(utime.ticks_ms()-init_2)   # put time into queue
#             pos_2.put(encoder_2.read())           # put position into queue
#             motor_2.set_duty_cycle(pwm)           # set new pwm
#         else:
#             motor_2.disable()
#             break
#         yield
#     return
# 
# # This code creates a share, a queue, and two tasks to enable the motors, then starts the tasks. The
# # tasks run until somebody presses ENTER, at which time the scheduler stops and
# # printouts show diagnostic information about the tasks, share, and queue.
# 
# # init queue
# time = cqueue.FloatQueue(250)
# pos = cqueue.FloatQueue(250)
# 
# # second init queue
# time_2 = cqueue.FloatQueue(250)
# pos_2 = cqueue.FloatQueue(250)
# 
# 
# # Motor init 2
# enable_pin_2 = pyb.Pin(pyb.Pin.board.PC1, pyb.Pin.OUT_PP)
# in1pin_2 = pyb.Pin.cpu.A0
# in2pin_2 = pyb.Pin.cpu.A1
# tim5 = pyb.Timer(5, freq=20000)
# motor_2 = MD.MotorDriver(enable_pin_2, in1pin_2, in2pin_2, tim5)
# motor_2.enable()
# 

# 
# # Encoder init 2
# pin_A = pyb.Pin.cpu.B6
# pin_B = pyb.Pin.cpu.B7
# tim4 = pyb.Timer(4, prescaler = 0, period = 2**16-1)
# encoder_2 = ER.Encoder(pin_A, pin_B, tim4)
# 
# # Run for 1
# fake = True
# while fake==True:
#     try:
#         kp = float(input("Enter a Kp value for 1:"))  # input for Kp
#         fake = False
#     except ValueError:
#         pass
#         
# sp1 = float(input("Enter a setpoint for 1:"))   # input for setpoint for run 1
# 
# # Run for 2
# kp_2 = float(input("Enter a Kp value for 2:"))  # input for Kp
# sp2 = float(input("Enter a setpoint for 2:"))   # input for setpoint for run 2
# 
# CL_2 = CLPC.ClosedLoop_P(kp_2,sp2) # use small Kp
# encoder_2.zero()  # zero encoder before using
# 
# CL = CLPC.ClosedLoop_P(kp,sp1) # use small Kp
# encoder.zero()  # zero encoder before using
# 
# # Create a share and a queue to test function and diagnostic printouts
# share0 = task_share.Share('h', thread_protect=False, name="Share 0")
# q0 = task_share.Queue('L', 16, thread_protect=False, overwrite=False,
#                       name="Queue 0")
# 
# # Create the tasks. If trace is enabled for any task, memory will be
# # allocated for state transition tracing, and the application will run out
# # of memory after a while and quit. Therefore, use tracing only for 
# # debugging and set trace to False when it's not needed
# task1 = cotask.Task(task1_fun, name="Task_1", priority=1, period=15,
#                     profile=True, trace=False, shares=(share0, q0))
# task2 = cotask.Task(task2_fun, name="Task_2", priority=1, period=15,
#                     profile=True, trace=False, shares=(share0, q0))
# cotask.task_list.append(task1)
# cotask.task_list.append(task2)
# 
# # Run the memory garbage collector to ensure memory is as defragmented as
# # possible before the real-time scheduler is started
# gc.collect()
# 
# # Run the scheduler with the chosen scheduling algorithm. Quit if ^C pressed
# while True:
#     if not time.full() == True:
#         cotask.task_list.pri_sched()
#     else:
#         break
# 
# # Disable the motors
# motor.disable()
# motor_2.disable()
# 
# # Print a table of task data and a table of shared information data
# # For loop to print and empty queue
# while True:  
#     print(f"{time.get()}, {pos.get()}")
#     if time.any() == False:
#         print("end")      # print end to indicate completion of data
#         motor.disable()   # turn off motor once data has been collected
#         break
# 
# while True:
#     print(f"{time_2.get()}, {pos_2.get()}")
#     if time_2.any() == False:
#         print("end")      # print end to indicate completion of data
#         motor_2.disable() # turn off motor once data has been collected
#         break
# 
# print('\n' + str (cotask.task_list))
# print(task_share.show_all())
# print(task1.get_trace())
# print('')
# 
# 
#
import utime as time
from machine import Pin, I2C
from mlx90640 import MLX90640
from mlx90640.calibration import NUM_ROWS, NUM_COLS, IMAGE_SIZE, TEMP_K
from mlx90640.image import ChessPattern, InterleavedPattern
import mlx_cam as MLX
from array import array
from math import tan, atan, pi
import motor_driver as MD
import encoder_reader as ER
import pyb
import CL_PID as PID
import Servo_Driver as SD

fire_time = time.ticks_ms()
# The following import is only used to check if we have an STM32 board such
    # as a Pyboard or Nucleo; if not, use a different library
try:
    from pyb import info

# Oops, it's not an STM32; assume generic machine.I2C for ESP32 and others
except ImportError:
    # For ESP32 38-pin cheapo board from NodeMCU, KeeYees, etc.
    i2c_bus = I2C(1, scl=Pin(22), sda=Pin(21))

# OK, we do have an STM32, so just use the default pin assignments for I2C1
else:
    i2c_bus = I2C(1)

print("MXL90640 Easy(ish) Driver Test")

# Select MLX90640 camera I2C address, normally 0x33, and check the bus
i2c_address = 0x33
scanhex = [f"0x{addr:X}" for addr in i2c_bus.scan()]
print(f"I2C Scan: {scanhex}")
cam_store = array('i')
array_store = []

# Create the camera object and set it up in default mode
camera = MLX.MLX_Cam(i2c_bus)

while True:
    try:
        # Get and image and see how long it takes to grab that image
        print("Click.", end='')
        begintime = time.ticks_ms()
        image = camera.get_image()
        print(f" {time.ticks_diff(time.ticks_ms(), begintime)} ms")

        # Can show image.v_ir, image.alpha, or image.buf; image.v_ir best?
        # Display pixellated grayscale or numbers in CSV format; the CSV
        # could also be written to a file. Spreadsheets, Matlab(tm), or
        # CPython can read CSV and make a decent false-color heat plot.
        show_image = False
        show_csv = True
        if show_image:
            camera.ascii_image(image.buf)
        elif show_csv:
            for line in camera.get_csv(image.v_ir, limits=(0, 99)):
                #print(line)
                for f in line.split(','):
                    #print(f)
                    cam_store.append(int(f))
                #print(range(cam_store)
                array_store.append(cam_store)
                #print(array_store)
                cam_store = array('i')
            break
        else:
            camera.ascii_art(image.v_ir)
        time.sleep_ms(10000)
        print('section done')

    except KeyboardInterrupt:
        break
    
max_value = float('-inf')  # Initialize with negative infinity to handle negative values
max_index = None

for row, line in enumerate(array_store):
    for column, value in enumerate(line):
        if value > max_value:
            max_value = value
            max_index = column

print("Maximum value:", max_value)
print("Index of maximum value:", max_index)

cam_angle = ((max_index-16)/32)*((55*pi)/180)
x = 6*tan(cam_angle)
beta = atan(x/((72+80.5)/12))
desired_pos = 144 + (beta/((1.25*pi)/180))

#servo init
servo_pin = pyb.Pin.cpu.B6

# Set period and prescaler to give freq = 50Hz
tim4 = pyb.Timer(4, prescaler = 79, period = 19999)
ch1 = tim4.channel(1, pyb.Timer.PWM, pin=servo_pin)

# Working cycle is from 500-2500microsecs, with 180 angle (from servo specs)
min_pw = 500
max_pw = 2500
angle_range = 180

# Create the servo driver
servo = SD.ServoDriver(ch1, min_pw, max_pw, angle_range)

# Motor init
enable_pin = pyb.Pin(pyb.Pin.board.PA10, pyb.Pin.OUT_PP)
in1pin = pyb.Pin.cpu.B4
in2pin = pyb.Pin.cpu.B5
tim3 = pyb.Timer(3, freq=20000)
motor = MD.MotorDriver(enable_pin, in1pin, in2pin, tim3)
motor.enable()

# Encoder init
pin_A = pyb.Pin.cpu.C6
pin_B = pyb.Pin.cpu.C7
tim8 = pyb.Timer(8, prescaler = 0, period = 2**16-1)
encoder = ER.Encoder(pin_A, pin_B, tim8)

# controller init
kp = 75#float(input("Enter a Kp value:"))  # input for Kp
ki = 0#float(input("Enter a Ki value:"))  
kd = 100#float(input("Enter a Kd value:"))


pid = PID.ClosedLoop_PID(kp,ki,kd,desired_pos,10/1000)
encoder.zero()
print(desired_pos//1)
while True:
    pwm = pid.run(encoder.read())      # set return from controller as pwm for motor
    motor.set_duty_cycle(pwm)         # set new pwm
    time.sleep_ms(10)
    if encoder.read() == desired_pos//1:
        print('done')
        break

motor.set_duty_cycle(0)
servo.set_angle(15)
utime.sleep_ms(35)
servo.set_angle(30)
utime.sleep_ms(100)
servo.set_angle(15)
utime.sleep_ms(50)
print('total run time', time.ticks_ms()-fire_time)
time.sleep(1)
motor.disable()    
print('test')
print(array_store)
print ("Done.")

## @endcond End the block which Doxygen should ignore

