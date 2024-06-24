from math import pi
import numpy as np
import logging
import sys
import time
import csv
import functions.BNOSensor as BNO
import functions.Vicon as Vicon

def init(bno, mytracker, object_name, CTRLR, error, mypi, pins, relay_pin):
    # Initial localization
    x, y, z = Vicon.GetLinearStates(mytracker, object_name, np.array([0,0,0]))
    yaw, pitch, roll, dyaw, droll, dpitch, a_x, a_y, a_z = BNO.getStates(bno,mypi,pins,relay_pin)
    yaw_looper = 0
    rawyaw = yaw
    cur_time = time.time() 
    state = np.transpose(np.array([[x, y, z, roll, pitch, yaw, 0, 0, 0, droll, dpitch, dyaw]]))
    # Create Setpoint
    target_height = .05 # 5cm or 0.05m
    setpoint = np.transpose(np.array([[x, y, z+target_height, 0, 0, 0, 0, 0, 0, 0, 0, 0]])) #setting to zero roll pitch yaw
    # Initialize Vicon filter states and parameters
    filter_states = [x, y, z, 0, 0, 0]
    filterparams = {"Tx" : .1, "Ty" : .1, "Tz" : .1, "Tdx" : .25, "Tdy" : .25, "Tdz" : .25, 
                    "Kx" : 1,  "Ky" : 1,  "Kz" : 1,  "Kdx" : 1,   "Kdy" : 1,   "Kdz" : 1}
    # PWM motor parameters
    PWMparams = {"vmax": 11.9, "RbkT": 1.29e-7 , "ke": 0.000656} 
    # Controller Parameters
    if CTRLR == "LQR":
        reader = csv.reader(open("ControlDesign/Controllers/LQRcontroller.csv", "r"), delimiter=",")
        K = list(reader)
        K = np.array(K).astype("float")
       	feedbackparams = {
       	    "K": K,
            "ue": np.transpose(4414.91*np.array([[1,1,1,1]]))
        }
    elif CTRLR ==  "PD":
        k = 1.29e-7 # rotor lift coefficient
        l = .23     # rotor distance from center of mass
        b = 8.21e-9 # rotor drag coefficient
        feedbackparams = {
            "K_x"     : 0,
            "K_y"     : 0,
            "K_z"     : 0,
            "K_dx"    : 0,
            "K_dy"    : 0,
            "K_dz"    : 0,
            "K_roll"  : .75,
            "K_pitch" : .75,
            "K_yaw"   : 0,
            "K_droll" : .25,
            "K_dpitch": .25,
            "K_dyaw"  : 0, 
            "K_motor" : 0,
            "mg"      : 13.52, #1.3785kg*9.81m/s^2 = 13.52N (with mounting plate)
            "Gamma"   : np.linalg.inv(np.array([[k, k, k, k], [0, l*k, 0, -l*k], [l*k, 0, -l*k, 0], [-b, b, -b, b]])),
            "sinYawSet" : np.sin(setpoint[5])/9.81,
            "cosYawSet" : np.cos(setpoint[5])/9.81
        }
    else:
        print("Ill-defined controller. Terminating program.")
        error = True
    return setpoint, state, cur_time, feedbackparams, PWMparams, filterparams, filter_states, yaw_looper, rawyaw, error

def FilterSignal(signal_in,dt,filter_state,T,K):
    signal_out = filter_state
    filter_state = (1-dt/T)*filter_state + K*dt/T*signal_in
    return signal_out, filter_state

def EstimateRates(x, y, z, dt, prev_state):
    dxdt = (x - prev_state[0]) / dt
    dydt = (y - prev_state[1]) / dt
    dzdt = (z - prev_state[2]) / dt
    return dxdt.item(), dydt.item(), dzdt.item()

def RectifyYaw(yaw,prev_yaw,yaw_looper):
    if yaw > prev_yaw + np.pi:
        # Assume yaw has increased past 2pi, so 2pi must be subtracted
        yaw_looper = yaw_looper - 2*np.pi
    elif yaw < prev_yaw - np.pi:
        # Assume yaw has decreased past 2pi, so 2pi must be added
        yaw_looper = yaw_looper + 2*np.pi
    yaw = yaw + yaw_looper
    return yaw, yaw_looper 

def SaveData(myfile, cur_time, state, inputs, dx, yaw_looper, rawyaw):
    save_vec = np.transpose(np.concatenate((np.array([[cur_time]]), state, inputs, dx, np.array([[yaw_looper]]), np.array([[rawyaw]])),axis=0))
    np.savetxt(myfile, save_vec, delimiter=',', fmt='%f')
    return 

def RectifyControl(PW):
    for index in range(4):
        if PW[index] > 1900:
            PW[index] = 1900
        elif PW[index] < 1110: # make it a little bit above zero so that the motor doesn't cut out entirely.
            PW[index] = 1110
    return PW  

def Speed2PW(w,p):
    PW = 800/p["vmax"]*(p["RbkT"]*np.power(w,2) + p["ke"]*w) + 1100
    return PW


