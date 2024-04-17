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

# Controller.py files
from Controller import EstimateRates
from Controller import CalculateControlAction_LQR
from Controller import CalculateControlAction_TestFeedback

state = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12]
inputs = CalculateControlAction_LQR(state)
print(inputs)

        







