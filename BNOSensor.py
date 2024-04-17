import logging
from math import pi
import sys
import time
import RPi.GPIO as GPIO
import ESC 

from Adafruit_BNO055 import BNO055

# Returns the states.
def getStates(bno,mypi,pins):
    try:
        yaw, roll, pitch = bno.read_euler()
        w_x, w_y, w_z = bno.read_gyroscope()
        a_x, a_y, a_z = bno.read_accelerometer()
        yaw = yaw/360*2*pi
        roll = roll/360*2*pi
        pitch = pitch/360*2*pi
        w_x = w_x/360*2*pi
        w_y = w_y/360*2*pi
        w_z = w_z/360*2*pi
    except:
        ESC.StopMotors(mypi,pins)
        print("IMU Disconnected. Setting motors to zero and terminating.")
        sys.exit()
    return yaw, roll, pitch, w_x, w_y, w_z, a_x, a_y, a_z

# Connects the BNO and prints general information.
def connectSensor(bno):
    # Enable verbose debug logging if -v is passed as a parameter.
    if len(sys.argv) == 2 and sys.argv[1].lower() == '-v':
        logging.basicConfig(level=logging.DEBUG)
    
    # Initialize the BNO055 and stop if something went wrong.
    if not bno.begin():
        raise RuntimeError('Failed to initialize BNO055! Is the sensor connected?')

    # Print system status and self test result.
    status, self_test, error = bno.get_system_status()
    print('System status: {0}'.format(status))
    print('Self test result (0x0F is normal): 0x{0:02X}'.format(self_test))
    # Print out an error if system status is in error mode.
    if status == 0x01:
        print('System error: {0}'.format(error))
        print('See datasheet section 4.3.59 for the meaning.')
    
    # Print BNO055 software revision and other diagnostic data.
    sw, bl, accel, mag, gyro = bno.get_revision()
    print('Software version:   {0}'.format(sw))
    print('Bootloader version: {0}'.format(bl))
    print('Accelerometer ID:   0x{0:02X}'.format(accel))
    print('Magnetometer ID:    0x{0:02X}'.format(mag))
    print('Gyroscope ID:       0x{0:02X}\n'.format(gyro))
    
# Callibrates the sensor.
def callibrateSensor(bno):
    print("Callibrating gyroscope...")
    print("Place drone flat for a few seconds.")
    print("Callibration completed when status = 3.")
    sys, gyro, accel, mag = bno.get_calibration_status()
    print("Calibration status: {}".format(gyro));
    if not gyro == 3:
        time.sleep(3)
    while not gyro == 3:
        time.sleep(1)
        sys, gyro, accel, mag = bno.get_calibration_status()
        print("Calibration status: {}".format(gyro));
    print("Gyroscope callibrated!\n\n")
    print("Callibrating accelerometer...")
    print("Place drone in six different positions. Include positions along x, y, and z axis.")
    print("Callibration completed when status = 3.")
    sys, gyro, accel, mag = bno.get_calibration_status()
    print("Calibration status: {}".format(accel));
    if not accel == 3:
        time.sleep(3)
    while not accel == 3:
        time.sleep(1)
        sys, gyro, accel, mag = bno.get_calibration_status()
        print("Calibration status: {}".format(accel));
    print("Accelerometer callibrated!\n\n")