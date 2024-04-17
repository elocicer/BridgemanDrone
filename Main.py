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

calibrate = False
CTRLR = "LQR"
error = False
if CTRLR == "LQR":
    from Feedback import LQR as CalculateControlAction
elif CTRLR ==  "PD":
    from Feedback import PD as CalculateControlAction
else:
    print("Ill-defined controller. Terminating program")
    sys.exit


bno, mytracker, object_name = Sensors.init(calibrate)

pins, mypi = ESC.init()

setpoint, state, cur_time, feedbackparams, PWMparams, filterparams, filter_states, yaw_looper, rawyaw, error = ctrl.init(bno, mytracker, object_name, CTRLR, error)

if error:
    sys.exit()

with open('data.csv', 'w', newline='') as myfile:
    try:
        while True:
            state, dx, cur_time, filter_states, yaw_looper, rawyaw = Sensors.getState(bno, mytracker, object_name, state, setpoint, cur_time, filter_states, filterparams, yaw_looper,rawyaw,mypi,pins)
            inputs          = CalculateControlAction(dx, feedbackparams, PWMparams, mypi, pins)
            ESC.writeMotors(mypi,pins,inputs)
            ctrl.SaveData(myfile, cur_time, state, inputs, dx, yaw_looper, rawyaw)
        
    except:
        pass
    
ESC.StopMotors(mypi,pins)
