# Main.py
# Ethan LoCicero and Alex Penne

# Import libraries
import numpy as np
import logging
import sys
import time
import subprocess
import csv
import RPi.GPIO as GPIO
import pigpio as pigpio
from Adafruit_BNO055 import BNO055
# Import functions
import Sensors
import BNOSensor as BNO
import Vicon
import ESC
import Controller as ctrl


pins, mypi = ESC.init()

# Controller loop.
with open('data.csv', 'w', newline='') as myfile:
    #csvwriter = csv.writer(myfile)
    try:
        while True:
            motor_number = input("Motor Number : ")
            PW           = input("Pulsewidth   : ")
            mypi.set_servo_pulsewidth(pins[int(motor_number)-1], float(PW))
        
    except KeyboardInterrupt:
        pass
    
ESC.StopMotors(mypi,pins)