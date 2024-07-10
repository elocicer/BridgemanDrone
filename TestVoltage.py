# Main.py
# Ethan LoCicero and Alex Penne

# Import libraries
import sys
import numpy as np
import logging
import time
import subprocess
import csv
import RPi.GPIO as GPIO
import pigpio as pigpio
from Adafruit_BNO055 import BNO055
# Import functions
import functions.Sensors as Sensors
import functions.BNOSensor as BNO
import functions.Vicon as Vicon
import functions.ESC as ESC
import functions.Controller as ctrl

import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

R1 = 18.25 #kOhm
R2 = 10 #kOhm
i2c = busio.I2C(board.SCL,board.SDA)
ads = ADS.ADS1115(i2c)
channel = AnalogIn(ads,ADS.P0)
print(f"Split Voltage: {channel.voltage:2.2f}V")
v_battery = channel.voltage * ((R1+R2)/R2)
print(f"Battery Voltage: {v_battery:2.2f}V")
