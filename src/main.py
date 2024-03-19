"""!
@file main.py

This file contains code to run all the necessary tasks to enable the Nerf gun
such that it is able to read an image, turn to the specified target, and pull the
trigger at the specified time.
This includes previously written code, including the motor driver class, the
encoder reader class, PID controller class, servo driver class, and the provided
mlx cam folder and classes.

@author Aaron Escamilla, Karen Morales De Leon, Joshua Tuttobene
@date   03/19/2024 Original program
@copyright (c) 2023 by Spluttflob and released under the GNU Public Licenes V3
"""

import cotask
import task_share
import utime as time
runtime = time.ticks_ms()
from machine import Pin, I2C
import mlx_cam as MLX
from math import tan, atan, pi
import motor_driver as MD
import encoder_reader as ER
import pyb
import CL_PID as PID
import Servo_Driver as SD
import gc
import cqueue

def task1_image(shares):
    """!
    Task 1 puts things into a share, specifically the desired position,
    a flag to mark whether the image is ready to be processed, and a camera share for data.
    Task 1 is in charge of utilizing the camera by reading the image, processing it into usable data
    and calculating the heat centroid. The task uses everything in the mlx90640 folder.
    @param shares A list holding the share and queue used by this task
    """
    print('task1_init')
    # ----[Camera Init]----
    begintime = time.ticks_ms()
    gc.collect()
    
    # Locate 'em state
    desired_pos, image_ready, camera = shares
    print('task1')

    # Keep trying to get an image; this could be done in a task, with
    # the task yielding repeatedly until an image is available
    while time.ticks_ms() - runtime < 5000:
        yield
    image = None
    while not image:
        image = camera.get_image_nonblocking()
        yield
    gc.collect()
    
    # Iterate over the camera data
    centroid = camera.run(40,image)
    print('centroid:',centroid,'angle:',((centroid-16)/32)*55)
    del camera, image

    # Get an image and see how long it takes to grab that image
    gc.collect()
    
    # ----[Camera Angle Calculation for Motor Control]----
    cam_angle = ((centroid-16)/32)*((55*pi)/180)
    x = 108*tan(cam_angle) + 12.1
    beta = atan(x/197.5)
    cal_offset = 0  # 2.5 to 3 off of center based on aiming for center
    desired = (144 + (beta/((1.25*pi)/180)))
    
    print('this is old', desired, (desired*1.25)-180)
    desired += cal_offset
    
    if desired - (desired//1) < 0.5:
        desired = desired//1
    elif desired - (desired//1) >= 0.5:
        desired = (desired//1) + 1
    
    pos_queue.put(int(desired))
    image_ready.put(1)
    yield
    while True:
        yield

def task2_motor(shares):
    """!
    Task 2 puts things into a share. Task 2 is for the motor initialization and it makes use of the
    motor driver class, PID class, and encoder class.
    It takes information from task 1 to determine
    where the motor should move via shares (desired position, and image_ready flag)
    @param shares A list holding the share and queue used by this task
    """
    print('task2_init')
    desired_pos, image_ready = shares
    # ----[Motor Init]----
    enable_pin = pyb.Pin(pyb.Pin.board.PA10, pyb.Pin.OUT_PP)
    in1pin = pyb.Pin.cpu.B4
    in2pin = pyb.Pin.cpu.B5
    tim3 = pyb.Timer(3, freq=20000)
    motor = MD.MotorDriver(enable_pin, in1pin, in2pin, tim3)
    motor.enable()

    # ----[Encoder Init]----
    pin_A = pyb.Pin.cpu.C6
    pin_B = pyb.Pin.cpu.C7
    tim8 = pyb.Timer(8, prescaler = 0, period = 2**16-1)
    encoder = ER.Encoder(pin_A, pin_B, tim8)
    encoder.zero()

    # ----[PID Controller Init]----
    kp = 17      
    ki = 0        
    kd = 15
    desired_pos = 144
    pid = PID.ClosedLoop_PID(kp,ki,kd,desired_pos,10/1000)
    
    print('task2')
    if not image_ready.get():
        while True:
            pwm = pid.run(encoder.read())  # set return from controller as pwm for motor
            motor.set_duty_cycle(pwm)                                 # set new pwm
            if encoder.read() == desired_pos or image_ready.get():
                motor.set_duty_cycle(0)
                break
            yield
    while True:
        if image_ready.get():
            break
        yield
        
    desired = pos_queue.get()
    print(desired)
    pid.set_setpoint(desired)
        
    # Target 'em state
    print('aim start')
    while True:
        pwm = pid.run(encoder.read())      # set return from controller as pwm for motor
        motor.set_duty_cycle(pwm)
        if encoder.read() == desired:
            motor.set_duty_cycle(0)
            fire_at_will.put(1)
            motor.disable()
            break
        yield

    while True:
        yield

def task3_servo(shares):
    """!
    Task 3 puts things into a share. Task 3 is for the encoder initialization and it makes use of the
    servo driver class. It is used to pull the trigger in the Nerf gun.
    It takes information from task 2 to determine whether the gun is in position to pull the trigger
    through a flag (fire_at_will). In addition, it sets the flag to reset everything in the next task.
    @param shares A list holding the share and queue used by this task
    """
    print('task3_init')
    # ----[Servo Init]----
    servo_pin = pyb.Pin.cpu.B6

    # Set period and prescaler to give freq = 50 Hz
    tim4 = pyb.Timer(4, prescaler = 79, period = 19999)
    ch1 = tim4.channel(1, pyb.Timer.PWM, pin=servo_pin)

    # Working cycle is from 500-2500 microsecs, with 180 angle (from servo specs)
    min_pw = 500
    max_pw = 2500
    angle_range = 180

    # Create the servo driver
    servo = SD.ServoDriver(ch1, min_pw, max_pw, angle_range)
    
    # Shoot 'em state
    fire_at_will, reset = shares
    print('task3')
    while True:
        if fire_at_will.get():
            servo.set_angle(30)     # 30 is good for new servo extension
            time.sleep_ms(200)
            reset.put(1)
            break
        yield
    servo.set_angle(15)
    while True:
        yield
    
    
def task4_reset(shares):
    """!
    Task 4 puts things into a share. Task 4 resets everything to its original  state such that the program
    is ready to run again. It takes information from task 3 with the reset flag and resets all the other
    variables from shares.
    @param shares A list holding the share and queue used by this task
    """
    # Do it to 'em again state
    reset, fire_at_will, desired_pos = shares
    print('task4')
    while True:
        if reset.get():
            print ("reset.")
            reset.put(0)
            fire_at_will.put(0)
            desired_pos.put(0)
            image_ready.put(0)
        yield
    while True:
        yield

# -------------------------[MAIN: RUNNING THE CODE]------------------------------
pos_queue = cqueue.IntQueue(1)
# Create a share and a queue to test function and diagnostic printouts
desired_pos = task_share.Share('h', thread_protect=False, name="Desired Position")
fire_at_will = task_share.Share('h', thread_protect=False, name="Fire Them")
reset = task_share.Share('h', thread_protect=False, name="Reset")
image_ready = task_share.Share('h', thread_protect=False, name="Image Ready")

# Initialize the camera
i2c_bus = I2C(1,freq = 1000000)
i2c_address = 0x33
scanhex = [f"0x{addr:X}" for addr in i2c_bus.scan()]
print(f"I2C Scan: {scanhex}")
gc.collect()
camera = MLX.MLX_Cam(i2c_bus)

# Create the tasks
task1 = cotask.Task(task1_image, name="Task_1", priority=3, period=110,
                profile=True, trace=False, shares=(desired_pos, image_ready,camera))
task2 = cotask.Task(task2_motor, name="Task_2", priority=4, period=15,
                profile=True, trace=False, shares=(desired_pos, image_ready))
task3 = cotask.Task(task3_servo, name="Task_3", priority=2, period=20,
                profile=True, trace=False, shares=(fire_at_will, reset))
task4 = cotask.Task(task4_reset, name="Task_4", priority=1, period=20,
                profile=True, trace=False, shares=(reset, fire_at_will, desired_pos))
cotask.task_list.append(task1)
cotask.task_list.append(task2)
cotask.task_list.append(task3)
cotask.task_list.append(task4)

# Run the memory garbage collector to ensure memory is defragmented
gc.collect()

while True:
    try:
        cotask.task_list.pri_sched()
    except KeyboardInterrupt:
        print('done')
        break
        
