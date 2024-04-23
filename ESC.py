import logging
import sys
import time
import RPi.GPIO as GPIO
import pigpio as pigpio
import subprocess
from Adafruit_BNO055 import BNO055

def init():
    # Set pin numbers and connect the motors.
    relay_pin = 25
    pins = [21, 18, 4, 13] # using GPIO.BCM numbering
    mypi = connectMotorsPigpio(pins, relay_pin)
    print("Motors connected and callibrated!")
    return pins, mypi

def connectMotorsPigpio(pins, relay_pin):
    print("Connecting motors...")
    '''
    This function automatically performs the ESC calibration procedure. 
    First, power is disconnected from the motors, and the ESC PWM signals are set to high.
    Then, power is connected, and the ESC PWM signals are set to low.
    At this point, the ESCs should beep 4 times (depending on the model) 
    to indicate the motors are calibrated and armed.
    The operator provides verification via keystroke to complete the procedure.
    '''

    # Min and max pulsewidth values
    maxval = 1900
    minval = 1100

    # Create rpi object
    mypi = pigpio.pi()
    
    # Ensure relay is disconnected and ESCs are set to high
    mypi.write(relay_pin, 0)
    time.sleep(2)
    mypi.write(relay_pin, 1)
    for pin in pins:
        mypi.set_servo_pulsewidth(pin, minval)
    time.sleep(2)
    
    # Connect relay
    for pin in pins:
        mypi.set_servo_pulsewidth(pin, minval)
    print("4 beeps indicates drone is armed. Otherwise, needs callibration.")
    print("End arming sequence.\n\n")
    input("Press any key to continue.")

    #
    print("Starting program. To stop drone, press Ctrl+C.")
    time.sleep(3)
    return mypi

def writeMotors(mypi,pins,inputs):
    for i in range(0,4):
                mypi.set_servo_pulsewidth(pins[i], inputs[i])
    return

def StopMotors(mypi,pins):
    mypi.write(relay_pin, 0)
    for pin in pins:
        mypi.set_servo_pulsewidth(pin, 1100)
    print("Motors Stopped.")
