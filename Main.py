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


calibrate = False
CTRLR = "PD"
error = False
if CTRLR == "LQR":
    from functions.Feedback import LQR as CalculateControlAction
elif CTRLR ==  "PD":
    from functions.Feedback import PD as CalculateControlAction
else:
    print("Ill-defined controller. Terminating program")
    sys.exit


bno, mytracker, object_name, VoltageDivider = Sensors.init(calibrate)

pins, mypi, relay_pin = ESC.init()

setpoint, state, cur_time, feedbackparams, PWMparams, filterparams, filter_states, yaw_looper, rawyaw, error = ctrl.init(bno, mytracker, object_name, CTRLR, error, mypi, pins, relay_pin)

if error:
    sys.exit()

with open('data.csv', 'w', newline='') as myfile:
    try:
        while True:
            state, dx, v_battery, cur_time, filter_states, yaw_looper, rawyaw = Sensors.getState(bno, mytracker, object_name, state, setpoint, cur_time, filter_states, filterparams, yaw_looper,rawyaw,mypi,pins,relay_pin,VoltageDivider,PWMparams)
            inputs          = CalculateControlAction(dx, v_battery, feedbackparams, PWMparams, mypi, pins)
            ESC.writeMotors(mypi,pins,inputs)
            ctrl.SaveData(myfile, cur_time, state, inputs, dx, yaw_looper, rawyaw, v_battery)
    except:
        pass
    
ESC.StopMotors(mypi,pins,relay_pin)
