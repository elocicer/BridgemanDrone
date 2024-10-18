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

# Calibrate IMU? (usually unnecessary)
calibrate = False

# Pick control type
CTRLR = "PD"
error = False
if CTRLR == "LQR":
    from functions.Feedback import LQR as CalculateControlAction
elif CTRLR ==  "PD":
    from functions.Feedback import PD as CalculateControlAction
else:
    print("Ill-defined controller. Terminating program")
    sys.exit

# Initialize sensors
bno, mytracker, object_name, ADS1115 = Sensors.init(calibrate)

# Initialize motors
pins, mypi, relay_pin = ESC.init()

# Initialize controller
setpoint, state, cur_time, feedbackparams, PWMparams, filterparams, filter_states, yaw_looper, rawyaw, error = ctrl.init(bno, mytracker, object_name, CTRLR, error, mypi, pins, relay_pin)

if error: # check the controller is well defined
    sys.exit()

# Main loop
with open('data.csv', 'w', newline='') as myfile:
    try:
        v_battery = 12.6
        while True: #v_battery > 10.0:
            # Get states
            state, dx, v_battery, cur_time, filter_states, yaw_looper, rawyaw = Sensors.getState(bno, mytracker, object_name, state, setpoint, cur_time, filter_states, filterparams, yaw_looper, rawyaw, mypi, pins, relay_pin, ADS1115)
            # Calculate inputs
            inputs = CalculateControlAction(dx, v_battery, feedbackparams, PWMparams, mypi, pins)
            # Write inputs to motors
            ESC.writeMotors(mypi,pins,inputs)
            # Save data
            ctrl.SaveData(myfile, cur_time, state, inputs, dx, yaw_looper, rawyaw, v_battery)
    except: # shut off motors on any error
        pass

ESC.StopMotors(mypi,pins,relay_pin)
