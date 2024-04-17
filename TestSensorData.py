# Main.py
# Ethan LoCicero and Alex Penne
# Run on Raspberry Pi to connect to 9DOF BNO055 sensor and quadcopter motors.
# Dependencies: BNOSensor.py, ESC.py

# Import needed libraries.
import numpy as np
import logging
import sys
import time
import subprocess
import RPi.GPIO as GPIO
import pigpio as pigpio
from Adafruit_BNO055 import BNO055
# Import functions from other files.
# BNOSensor.py files.
from BNOSensor import connectSensor
from BNOSensor import callibrateSensor
from BNOSensor import getStates
# ESC.py files.
from ESC import connectMotors # Don't use this one for now. Software pwm. 
from ESC import connectMotorsPigpio
# Vicon.py files
from Vicon import connectVicon
from Vicon import GetLinearStates
# Controller.py files
from Controller import EstimateRates
from Controller import CalculateControlAction_LQR
from Controller import CalculateControlAction_TestFeedback



maxval = 1900
minval = 1100
# CONNECT AND CALLIBRATE
# Raspberry Pi configuration with serial UART and RST connected to GPIO 18:
# UART mode must be turned on (PS1 pin = HIGH).
bno = BNO055.BNO055(serial_port='/dev/serial0', rst=18)
# Connect the sensor.
connectSensor(bno)
print("Sensor connected!")
# Callibrate the sensor.
#callibrateSensor(bno)
print("Sensor callibrated!\n\n")
# Connect to Vicon
vicon_client, mytracker = connectVicon("192.168.0.101")
OBJECT_NAME = "LoCicero_RPI_Drone"

# Get Object position
position = mytracker.get_position(OBJECT_NAME) # (latency, frame number, [[object_name,object_name,x,y,z,roll,pitch,yaw]]) (mm, rad)
print(f"Position: {position}")
time.sleep(2)



# Set pin numbers and connect the motors.
pins = [24, 26, 17, 16] # using GPIO.BCM numbering
mypi = connectMotorsPigpio(pins)
print("Motors connected and callibrated!")


# START PROGRAM
print("Starting program. To kill drone, kill the program using Ctrl+C.")
# Turn on motors.


# get initial position from VICON.
init_x, init_y, init_z = GetLinearStates(mytracker, OBJECT_NAME) 
# get initial position from BNO
init_yaw, init_roll, init_pitch, init_w_x, init_w_y, init_w_z, init_a_x, init_a_y, init_a_z = getStates(bno)
# Set setpoint (1 ft above initial position)
setpoint = [init_x, init_y, init_z+.3, init_roll, init_pitch, init_yaw, 0, 0, 0, 0, 0, 0]
# Get initial time.
init_time = time.time()
# Make previous state vector.
prev_state = [init_x, init_y, init_z, init_time]
# Controller loop.ß
try:
    while True:
        # Get x, y, z from VICON.
        x, y, z = GetLinearStates(mytracker, OBJECT_NAME)
        # Get current time.
        cur_time = time.time()
        # Estimate rates.
        dxdt, dydt, dzdt = EstimateRates(x, y, z, cur_time, prev_state)
        # Get attitude and rates from sensor.
        yaw, roll, pitch, w_x, w_y, w_z, a_x, a_y, a_z = getStates(bno)
        # Make state vector.
        state = [x, y, z, roll, pitch, yaw, dxdt, dydt, dzdt, w_x, w_y, w_z]
        state_array =np.array([[state[0]],[state[1]],[state[2]],[state[3]],[state[4]],[state[5]],[state[6]],[state[7]],[state[8]],[state[9]],[state[10]],[state[11]]])
        setpoint_array = np.array([[setpoint[0]],[setpoint[1]],[setpoint[2]],[setpoint[3]],[setpoint[4]],[setpoint[5]],[setpoint[6]],[setpoint[7]],[setpoint[8]],[setpoint[9]],[setpoint[10]],[setpoint[11]]])
        dx = state_array-setpoint_array
        print(dx)
        time.sleep(5)
        # Make current state the previous.                
        prev_state = [x, y, z, cur_time]
        
except KeyboardInterrupt: # This should allow us to exit the while loop by pressing Ctrl+C
    pass
    
    
# Sets drone to zero speed at end of program.
for pin in pins:
    mypi.set_servo_pulsewidth(pin, 1100)







### EXAMPLE CODES. Remove triple quotations to run. 

# Turn on max speed and constantly ask for next speed.
"""
for pin in pins:
    mypi.set_servo_pulsewidth(pin, maxval)
while True:
    speed = input("enter speed: ")
    for pin in pins:
        mypi.set_servo_pulsewidth(pin, speed)
"""
# Print yaw.
"""
while True:
    yaw, roll, pitch, w_x, w_y, w_z, a_x, a_y, a_z = getStates(bno)
    print(yaw)
"""







