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

relay_pin = 25
mypi = pigpio.pi()

while True:
    pinVal = input("Enter relay state: ")
    mypi.write(relay_pin, pinVal) # turns relay on or off 
