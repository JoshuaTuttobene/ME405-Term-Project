# # 
# import utime as time
# from machine import Pin, I2C
# import mlx_cam as MLX
# from math import tan, atan, pi
# import motor_driver as MD
# import encoder_reader as ER
# import pyb
# import CL_PID as PID
# import Servo_Driver as SD
# import gc
# 
# 
# # 
# fire_time = time.ticks_ms()
# # # The following import is only used to check if we have an STM32 board such
# #     # as a Pyboard or Nucleo; if not, use a different library
# gc.collect()
# 
# i2c_bus = I2C(1,freq = 1000000)
# 
# print("MXL90640 Easy(ish) Driver Test")
# 
# # Select MLX90640 camera I2C address, normally 0x33, and check the bus
# i2c_address = 0x33
# scanhex = [f"0x{addr:X}" for addr in i2c_bus.scan()]
# print(f"I2C Scan: {scanhex}")
# gc.collect()
# begintime = time.ticks_ms()
# 
# # Create the camera object and set it up in default mode
# camera = MLX.MLX_Cam(i2c_bus)
# print(f" {time.ticks_diff(time.ticks_ms(), begintime)} ms")
# begintime = time.ticks_ms()
# image = camera.get_image_nonblocking()
# print(f" {time.ticks_diff(time.ticks_ms(), begintime)} ms")
# 
# #Iterate over the camera data
# begintime = time.ticks_ms()
# centroid = camera.run(60,image)
# print(f" {time.ticks_diff(time.ticks_ms(), begintime)} ms")
# # print(val)
# # sums = 0
# # for x in range(32):
# #     sums += (x+1)*val[x]
# #     
# #     
# # centroid = sums / sum(val)
# 
# print('centroid',centroid,'angle',((centroid-16)/32)*55)
# 
# del camera
# 
# # Get and image and see how long it takes to grab that image
# print("Click.", end='')
# 
# gc.collect()
# 
# 
# 
# 
# # for center of table
# # cam_angle = ((max_index-16)/32)*((55*pi)/180)
# # x = 6*tan(cam_angle)
# # beta = atan(x/((108+89.5)/12))
# # cal_offset = 8
# # desired_pos = (144 + (beta/((1.25*pi)/180)))
# # print('this is old', desired_pos, (desired_pos*1.25)-180)
# # desired_pos += cal_offset
# 
# # calibrated from edge
# cam_angle = ((centroid-16)/32)*((55*pi)/180)
# x = 108*tan(cam_angle) + 7.85
# beta = atan(x/197.5)
# cal_offset = 4 # 2.5 to 3 off of center based on aiming for center
# desired_pos = (144 + (beta/((1.25*pi)/180)))
# 
# print('this is old', desired_pos, (desired_pos*1.25)-180)
# desired_pos += cal_offset
# 
# if desired_pos - (desired_pos//1) < 0.5:
#     desired_pos = desired_pos//1
# elif desired_pos - (desired_pos//1) >= 0.5:
#     desired_pos = (desired_pos//1) + 1
# 
# 
# # Servo Init
# servo_pin = pyb.Pin.cpu.B6
# 
# # Set period and prescaler to give freq = 50Hz
# tim4 = pyb.Timer(4, prescaler = 79, period = 19999)
# ch1 = tim4.channel(1, pyb.Timer.PWM, pin=servo_pin)
# 
# # Working cycle is from 500-2500microsecs, with 180 angle (from servo specs)
# min_pw = 500
# max_pw = 2500
# angle_range = 180
# 
# # Create the servo driver
# servo = SD.ServoDriver(ch1, min_pw, max_pw, angle_range)
# 
# # Motor init
# enable_pin = pyb.Pin(pyb.Pin.board.PA10, pyb.Pin.OUT_PP)
# in1pin = pyb.Pin.cpu.B4
# in2pin = pyb.Pin.cpu.B5
# tim3 = pyb.Timer(3, freq=20000)
# motor = MD.MotorDriver(enable_pin, in1pin, in2pin, tim3)
# motor.enable()
# 
# # Encoder init
# pin_A = pyb.Pin.cpu.C6
# pin_B = pyb.Pin.cpu.C7
# tim8 = pyb.Timer(8, prescaler = 0, period = 2**16-1)
# encoder = ER.Encoder(pin_A, pin_B, tim8)
# 
# # Controller init
# kp = 15      # float(input("Enter a Kp value:"))  # input for Kp
# ki = 0       # float(input("Enter a Ki value:"))  
# kd = 15     # float(input("Enter a Kd value:"))
# 
# 
# pid = PID.ClosedLoop_PID(kp,ki,kd,desired_pos,10/1000)
# encoder.zero()
# print(desired_pos,(desired_pos*1.25)-180)
# while True:
#     pwm = pid.run(encoder.read())      # set return from controller as pwm for motor
#     motor.set_duty_cycle(pwm)          # set new pwm
#     time.sleep_ms(10)
#     if encoder.read() == desired_pos:
#         motor.set_duty_cycle(0)
#         print('done')
#         break
# 
# print(desired_pos)
# # Run Servo
# servo.set_angle(15)
# time.sleep_ms(100)
# servo.set_angle(30)
# 
# 
# print('total run time', time.ticks_ms()-fire_time)
# print(encoder.read())
# 
# time.sleep(3)
# servo.set_angle(15)
# pid.set_setpoint(0)
# while True:
#     pwm = pid.run(encoder.read())      # set return from controller as pwm for motor
#     motor.set_duty_cycle(pwm)          # set new pwm
#     time.sleep_ms(10)
#     if encoder.read() == 0:
#         motor.set_duty_cycle(0)
#         print('done')
#         break
# motor.disable()
# 
# print('test')
# #print(array_store)
# print(encoder.read())
# print ("Done.")
# # 
# # ## @endcond End the block which Doxygen should ignore
# 
# # Camera code delete everything after here
# # while True:
# #     try:
# #         # Get and image and see how long it takes to grab that image
# #         print("Click.", end='')
# #         begintime = time.ticks_ms()
# #         image = camera.get_image()
# # 
# #         print(f" {time.ticks_diff(time.ticks_ms(), begintime)} ms")
# # 
# #         # Can show image.v_ir, image.alpha, or image.buf; image.v_ir best?
# #         # Display pixellated grayscale or numbers in CSV format; the CSV
# #         # could also be written to a file. Spreadsheets, Matlab(tm), or
# #         # CPython can read CSV and make a decent false-color heat plot.
# #         show_image = True
# #         show_csv = False
# #         if show_image:
# #             camera.ascii_image(image.buf)
# #         elif show_csv:
# #             for line in camera.get_csv_test(45, image.v_ir, limits=(0, 99)):
# #                 print(line)
# #         
# #         else:
# #             camera.ascii_art(image.v_ir)
# #         time.sleep_ms(10000)
# # 
# #     except KeyboardInterrupt:
# #         break
# # 
# # print ("Done.")
# # 
# # # ## @endcond End the block which Doxygen should ignore

# 
import utime as time
from machine import Pin, I2C
import mlx_cam as MLX
from math import tan, atan, pi
import motor_driver as MD
import encoder_reader as ER
import pyb
import CL_PID as PID
import Servo_Driver as SD
from array import array
import gc
# 
fire_time = time.ticks_ms()
# # The following import is only used to check if we have an STM32 board such
#     # as a Pyboard or Nucleo; if not, use a different library
gc.collect()

i2c_bus = I2C(1,freq = 1000000)

print("MXL90640 Easy(ish) Driver Test")

# Select MLX90640 camera I2C address, normally 0x33, and check the bus
i2c_address = 0x33
scanhex = [f"0x{addr:X}" for addr in i2c_bus.scan()]
print(f"I2C Scan: {scanhex}")
begintime = time.ticks_ms()
gc.collect()

# Create the camera object and set it up in default mode
camera = MLX.MLX_Cam(i2c_bus)
image = None
while not image:
    image = camera.get_image_nonblocking()
    time.sleep_ms(50)
max_value = 0
max_index = 0
gc.collect()
#Iterate over the camera data
val = [0]*32
for line in camera.get_csv(image, limits=(0, 99)):
    for column,value in enumerate(line.split(',')):
        val[column] += int(value)

for x in range(32):
    sums = x*val[x]
centroid = sums / sum(val)

print('centroid',centroid,'angle',((centroid-16)/32)*55)

del camera, image

# Get and image and see how long it takes to grab that image
print("Click.", end='')

gc.collect()


print(f" {time.ticks_diff(time.ticks_ms(), begintime)} ms")

# for center of table
# cam_angle = ((max_index-16)/32)*((55*pi)/180)
# x = 6*tan(cam_angle)
# beta = atan(x/((108+89.5)/12))
# cal_offset = 8
# desired_pos = (144 + (beta/((1.25*pi)/180)))
# print('this is old', desired_pos, (desired_pos*1.25)-180)
# desired_pos += cal_offset

# calibrated from edge
cam_angle = ((centroid-16)/32)*((55*pi)/180)
x = 108*tan(cam_angle) + 7.85
beta = atan(x/197.5)
cal_offset = 2.5 # 2.5 to 3 off of center based on aiming for center
desired_pos = (144 + (beta/((1.25*pi)/180)))

print('this is old', desired_pos, (desired_pos*1.25)-180)
desired_pos += cal_offset

if desired_pos - (desired_pos//1) < 0.5:
    desired_pos = desired_pos//1
elif desired_pos - (desired_pos//1) >= 0.5:
    desired_pos = (desired_pos//1) + 1


# Servo Init
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

# Controller init
kp = 15      # float(input("Enter a Kp value:"))  # input for Kp
ki = 0       # float(input("Enter a Ki value:"))  
kd = 15     # float(input("Enter a Kd value:"))


pid = PID.ClosedLoop_PID(kp,ki,kd,desired_pos,10/1000)
encoder.zero()
print(desired_pos,(desired_pos*1.25)-180)
while True:
    pwm = pid.run(encoder.read())      # set return from controller as pwm for motor
    motor.set_duty_cycle(pwm)          # set new pwm
    time.sleep_ms(10)
    if encoder.read() == desired_pos:
        motor.set_duty_cycle(0)
        print('done')
        break

print(desired_pos)
# Run Servo
servo.set_angle(15)
time.sleep_ms(100)
servo.set_angle(30)


print('total run time', time.ticks_ms()-fire_time)
print(encoder.read())

time.sleep(3)
servo.set_angle(15)
pid.set_setpoint(0)
while True:
    pwm = pid.run(encoder.read())      # set return from controller as pwm for motor
    motor.set_duty_cycle(pwm)          # set new pwm
    time.sleep_ms(10)
    if encoder.read() == 0:
        motor.set_duty_cycle(0)
        print('done')
        break
motor.disable()

print('test')
#print(array_store)
print(encoder.read())
print ("Done.")
# 
# ## @endcond End the block which Doxygen should ignore

# Camera code delete everything after here
# while True:
#     try:
#         # Get and image and see how long it takes to grab that image
#         print("Click.", end='')
#         begintime = time.ticks_ms()
#         image = camera.get_image()
#         print(f" {time.ticks_diff(time.ticks_ms(), begintime)} ms")
# 
#         # Can show image.v_ir, image.alpha, or image.buf; image.v_ir best?
#         # Display pixellated grayscale or numbers in CSV format; the CSV
#         # could also be written to a file. Spreadsheets, Matlab(tm), or
#         # CPython can read CSV and make a decent false-color heat plot.
#         show_image = True
#         show_csv = False
#         if show_image:
#             camera.ascii_image(image.buf)
#         elif show_csv:
#             for line in camera.get_csv(image.v_ir, limits=(0, 99)):
#                 print(line)
#         else:
#             camera.ascii_art(image.v_ir)
#         time.sleep_ms(10000)
# 
#     except KeyboardInterrupt:
#         break
# 
# print ("Done.")
# 
# ## @endcond End the block which Doxygen should ignore


