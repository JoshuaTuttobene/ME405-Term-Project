
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

fire_time = time.ticks_ms()
# The following import is only used to check if we have an STM32 board such
    # as a Pyboard or Nucleo; if not, use a different library
gc.collect()

i2c_bus = I2C(1)

print("MXL90640 Easy(ish) Driver Test")

# Select MLX90640 camera I2C address, normally 0x33, and check the bus
i2c_address = 0x33
scanhex = [f"0x{addr:X}" for addr in i2c_bus.scan()]
print(f"I2C Scan: {scanhex}")
begintime = time.ticks_ms()
gc.collect()

# Create the camera object and set it up in default mode
camera = MLX.MLX_Cam()
image = camera.get_image()
max_value = 0
max_index = 0

#Iterate over the camera data
sums = [0]*32
for line in camera.get_csv(image.v_ir, limits=(0, 99)):
    for column,value in enumerate(line.split(',')):
        sums[column] += int(value)
max_value = max(sums)
max_index = sums.index(max_value)



# Get and image and see how long it takes to grab that image
print("Click.", end='')

gc.collect()

print(max_value,max_index)

print(f" {time.ticks_diff(time.ticks_ms(), begintime)} ms")

cam_angle = ((max_index-16)/32)*((55*pi)/180)
x = 6*tan(cam_angle)
beta = atan(x/((108+80.5)/12))
cal_offset = 8
desired_pos = (144 + (beta/((1.25*pi)/180)))
print('this is old', desired_pos, (desired_pos*1.25)-180)
desired_pos += cal_offset

if desired_pos - (desired_pos//1) < 0.5:
    desired_pos = desired_pos//1
elif desired_pos - (desired_pos//1) > 0.5:
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
kp = 25      # float(input("Enter a Kp value:"))  # input for Kp
ki = 0       # float(input("Enter a Ki value:"))  
kd = 100     # float(input("Enter a Kd value:"))


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


# Run Servo
servo.set_angle(15)
time.sleep_ms(100)
servo.set_angle(34)


print('total run time', time.ticks_ms()-fire_time)

time.sleep(3)
servo.set_angle(15)
pid.set_setpoint(0)
while True:
    pwm = pid.run(encoder.read())      # set return from controller as pwm for motor
    motor.set_duty_cycle(pwm)          # set new pwm
    time.sleep_ms(10)
    if encoder.read() == 0:
        time.sleep_ms(50)
        motor.set_duty_cycle(0)
        print('done')
        break
motor.disable()

print('test')
#print(array_store)
print ("Done.")

## @endcond End the block which Doxygen should ignore

