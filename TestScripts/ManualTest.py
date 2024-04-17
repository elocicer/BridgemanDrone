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
'''
# BNOSensor.py files.
from BNOSensor import connectSensor
from BNOSensor import callibrateSensor
from BNOSensor import getStates
'''
# ESC.py files.
from ESC import connectMotors # Don't use this one for now. Software pwm. 
from ESC import connectMotorsPigpio
'''
# Vicon.py files
from Vicon import connectVicon
# Controller.py files
from Controller import EstimateRates
from Controller import CalculateControlActionLQR
'''


maxval = 1900
minval = 1100
'''
# CONNECT AND CALLIBRATE
# Raspberry Pi configuration with serial UART and RST connected to GPIO 18:
# UART mode must be turned on (PS1 pin = HIGH).
bno = BNO055.BNO055(serial_port='/dev/serial0', rst=18)
# Connect the sensor.
#connectSensor(bno)
print("Sensor connected!")
# Callibrate the sensor.
#callibrateSensor(bno)
print("Sensor callibrated!\n\n")
# Connect to Vicon
vicon_client, mytracker = connectVicon("192.168.0.101")
OBJECT_NAME = "LoCicero_test_box_2"

# Get Object position
position = mytracker.get_position(OBJECT_NAME) # (latency, frame number, [[object_name,object_name,x,y,z,roll,pitch,yaw]]) (mm, rad)
print(f"Position: {position}")
time.sleep(2)
'''


# Set pin numbers and connect the motors.
pins = [24, 26, 17, 16] # using GPIO.BCM numbering
mypi = connectMotorsPigpio(pins)
print("Motors connected and callibrated!")


# START PROGRAM
print("Starting program. To kill drone, kill the program using Ctrl+C.")
# Turn on motors.

'''
# Set setpoint.
setpoint = [0, 0, .25, 0, 0, 0, 0, 0, 0, 0, 0, 0]

positions = npp.array()
np.append(mytracker.get_position(OBJECT_NAME)) # Sets first position. Need this to get rate.

times = np.array()
np.append(times, time.time()) # Gets first time. Need this to get rate.

inputs = np.array()
'''

# Controller loop.
try:
    while True:
        speed = input("enter speed: ")
        for pin in pins:
            mypi.set_servo_pulsewidth(pin, speed)
except KeyboardInterrupt: # This should allow us to exit the while loop by pressing Ctrl+C
    pass
    
    
# Sets drone to zero speed at end of program.
for pin in pins:
    mypi.set_servo_pulsewidth(pin, 1100)







### EXAMPLE CODES. Remove triple quotations to run. 

# Turn on max speed and constantly ask for next speed.
"""

"""
# Print yaw.
"""
while True:
    yaw, roll, pitch, w_x, w_y, w_z, a_x, a_y, a_z = getStates(bno)
    print(yaw)
"""







