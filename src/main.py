
# Imports for cotasks
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
    Task 1 puts things into a share. Task 1 is for the first motor we use.
    It uses the motor class, encoder class,
    and the closed loop proportional controller class
    @param shares A list holding the share and queue used by this task
    """
    print('task1_init')
    # ----[Camera Init]----
    # Create the camera object and set it up in default mode


    # MLX90640 camera I2C address at 0x33, and check the bus
    begintime = time.ticks_ms()
    gc.collect()
    
    # Locate 'em state
    desired_pos, image_ready, camera = shares
    print('task1')

    # Keep trying to get an image; this could be done in a task, with
    # the task yielding repeatedly until an image is available
    while time.ticks_ms() - runtime < 4500:
        yield
    image = None
    while not image:
        image = camera.get_image_nonblocking()
        yield
    gc.collect()
    
    # Iterate over the camera data
    centroid = camera.run(50,image)
    #print('centroid:',centroid,'angle:',((centroid-16)/32)*55)
    del camera, image

    # Get an image and see how long it takes to grab that image
    #print("Click.", end='')
    gc.collect()
    
    # ----[Camera Angle Calculation for Motor Control]----
    
    # Location Code for Center of Table
    # cam_angle = ((max_index-16)/32)*((55*pi)/180)
    # x = 6*tan(cam_angle)
    # beta = atan(x/((108+89.5)/12))
    # cal_offset = 8
    # desired_pos = (144 + (beta/((1.25*pi)/180)))
    # print('this is old', desired_pos, (desired_pos*1.25)-180)
    # desired_pos += cal_offset
    
    # Location Code from Edge
    cam_angle = ((centroid-16)/32)*((55*pi)/180)
    x = 108*tan(cam_angle) + 12.1
    beta = atan(x/197.5)
    cal_offset = 0 # 2.5 to 3 off of center based on aiming for center
    desired = (144 + (beta/((1.25*pi)/180)))
    
    #print('this is old', desired, (desired*1.25)-180)
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
    Task 2 puts things into a share. Task 2 is for the motor to 
    activate the trigger.
    It uses the servo class
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
        #print('start 144')
        start_time = time.ticks_ms()
        while True:
            pwm = pid.run(encoder.read(),time.ticks_ms()-start_time)      # set return from controller as pwm for motor
            motor.set_duty_cycle(pwm)          # set new pwm
            #print(pwm,encoder.read())
            if encoder.read() == desired_pos or image_ready.get():
                motor.set_duty_cycle(0)
                break
            yield
    while True:
        #print('start hold')
        if image_ready.get():
            break
        yield
        
    desired = pos_queue.get()
    print(desired)
    pid.set_setpoint(desired)
        
    # Target 'em state
    print('aim start')
    start_time = time.ticks_ms()
    while True:
        pwm = pid.run(encoder.read(),time.ticks_ms()-start_time)      # set return from controller as pwm for motor
        motor.set_duty_cycle(pwm)
        #print(encoder.read(),',',desired,',',pwm)
        if encoder.read() == desired:
            motor.set_duty_cycle(0)
            fire_at_will.put(1)
            motor.disable()
            break
        yield
    #print(desired,(desired*1.25)-180)

    while True:

        yield

def task3_servo(shares):
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
    #print(fire_at_will)
    while True:
        if fire_at_will.get():
            servo.set_angle(40) # 30 is good for new servo extension

            time.sleep_ms(200)
            reset.put(1)
            break
        yield
    servo.set_angle(15)
    while True:
        yield
    
    
def task4_reset(shares):
    reset, fire_at_will, desired_pos = shares
    # Do it to 'em again state
    print('task4')
    while True:
        if reset.get():
            #print("first", reset)
    
            #print('test')
            print ("reset.")
            reset.put(0)
            #print("second", reset)
            fire_at_will.put(0)
            desired_pos.put(0)
            image_ready.put(0)
        yield
    while True:
        yield

# -------------------------[TEST CODE]------------------------------
#print('main')
pos_queue = cqueue.IntQueue(1)
# Create a share and a queue to test function and diagnostic printouts
desired_pos = task_share.Share('h', thread_protect=False, name="Desired Position")
fire_at_will = task_share.Share('h', thread_protect=False, name="Fire Them")
reset = task_share.Share('h', thread_protect=False, name="Reset")
image_ready = task_share.Share('h', thread_protect=False, name="Image Ready")

i2c_bus = I2C(1,freq = 1000000)
i2c_address = 0x33
scanhex = [f"0x{addr:X}" for addr in i2c_bus.scan()]
print(f"I2C Scan: {scanhex}")
gc.collect()
camera = MLX.MLX_Cam(i2c_bus)


# Create the tasks. If trace is enabled for any task, memory will be
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

# Run the memory garbage collector to ensure memory is as defragmented as
gc.collect()

while True:
    try:
        cotask.task_list.pri_sched()
    except KeyboardInterrupt:
        print('done')
        break
        
