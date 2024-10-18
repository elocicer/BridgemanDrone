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

#import board
#import busio
#import adafruit_ads1x15.ads1115 as ADS
#from adafruit_ads1x15.analog_in import AnalogIn

import smbus
import time

# define I2C bus
bus = smbus.SMBus(1)

# ADS1115 I2C address
address = 0x48

# config register address
config_register = 0x01

# Set range to +/-6.144V, samples per second to 860, differential comparison between AIN0 and AIN1, and continuous mode
config = 0x4083

# Write PGA configuration to ADS1115
bus.write_i2c_block_data(address, config_register, [(config >> 8) & 0xFF, config & 0xFF])

# Wait to take effect
time.sleep(0.01)

# Read register
conversion_register = 0x00
data = bus.read_i2c_block_data(address,conversion_register,2)

raw_adc = data[0] << 8 | data[1]
if raw_adc > 32767:
     raw_adc -= 65536
v_channel = raw_adc * 6.144 / 32768.0

R1 = 16.333 #kOhm (measured directly to 16.29, then tuned to fit in actual implementation)
R2 = 9.98 #kOhm
#i2c = busio.I2C(board.SCL,board.SDA)
#ads = ADS.ADS1115(i2c)
#channel = AnalogIn(ads,ADS.P0)
#v_channel = channel.voltage
v_battery = v_channel * ((R1+R2)/R2)
print(f"Raw ADC Value  : {hex(raw_adc)}")
print(f"Channel Voltage: {v_channel:2.6f}V")
print(f"Battery Voltage: {v_battery:2.6f}V")
