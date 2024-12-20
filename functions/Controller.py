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
    target_height = .2 # meters
    setpoint = np.transpose(np.array([[x, y, z+target_height, 0, 0, yaw, 0, 0, 0, 0, 0, 0]])) #setting to zero roll pitch yaw
    # Initialize Vicon filter states and parameters
    filter_states = [x, y, z, roll, pitch, yaw, 0, 0, 0, 0, 0, 0]
    filterparams = {"Tx"     : .1,  "Ty"      : .1,  "Tz"    : .1,
                    "Troll"  : .1,  "Tpitch"  : .1,  "Tyaw"  : .1,
                    "Tdx"    : .25, "Tdy"     : .25, "Tdz"   : .25, 
                    "Tdroll" : .25, "Tdpitch" : .25, "Tdyaw" : .25, 
                    "Kx"     : 1,   "Ky"      : 1,   "Kz"    : 1, 
                    "Kroll"  : 1,   "Kpitch"  : 1,   "Kyaw"  : 1,
                    "Kdx"    : 1,   "Kdy"     : 1,   "Kdz"   : 1,
                    "Kdroll" : 1,   "Kdpitch" : 1,   "Kdyaw" : 1,}
    # PWM motor parameters
    PWMparams = {"RbkT": 1.29e-7 , "ke": 0.000656} # RbkT refers to R*b/k_T.
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
        g = 9.81    # gravity, m/s^2
        m_measured = 1.0855  # mass, kg, measured (without mounting plate)
        m_adjustment = .1 # heuristic adjustment to fix offset during free flight
        m = m_measured + m_adjustment
        m_with_stand = 1.3 # mass with test stand
        m = m_with_stand
        feedbackparams = {
            "K_x"     : 0,
            "K_y"     : 0,
            "K_z"     : 0,
            "K_dx"    : 0,
            "K_dy"    : 0,
            "K_dz"    : 0,
            "K_roll"  : .8,
            "K_pitch" : .8,
            "K_yaw"   : .1,
            "K_droll" : 7.5, #7.5
            "K_dpitch": 7.5,
            "K_dyaw"  : .1,
            "K_motor" : 0,
            "mg"      : m*g,
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

def SaveData(myfile, cur_time, state, inputs, dx, yaw_looper, rawyaw, v_battery):
    save_vec = np.transpose(np.concatenate((np.array([[cur_time]]), state, inputs, dx, np.array([[yaw_looper]]), np.array([[rawyaw]]), np.array([[v_battery]])), axis=0))
    np.savetxt(myfile, save_vec, delimiter=',', fmt='%f')
    return 

def RectifyControl(PW):
    for index in range(4):
        if PW[index] > 1900:
            PW[index] = 1900
        elif PW[index] < 1110: # make it a little bit above zero so that the motor doesn't cut out entirely.
            PW[index] = 1110
    return PW  

def Speed2PW(w,v_battery,p):
    PW = 800/v_battery*(p["RbkT"]*np.power(w,2) + p["ke"]*w) + 1100
    return PW


