import numpy as np
import functions.Controller as ctrl
import functions.ESC as ESC
import sys

def LQR(dx, v_battery, FBp, PWMp, mypi, pins):
    try:
        u = FBp["ue"] - np.matmul(FBp["K"],dx)
        PW = ctrl.Speed2PW(u,v_battery,PWMp)
        PW = ctrl.RectifyControl(PW)
    except:
        ESC.StopMotors(mypi,pins)
        print("Feedback calculation error. Setting motors to zero and terminating.")
        sys.exit()
    return PW

def PD(dx, v_battery, FBp, PWMp, mypi, pins):
    # based on Mahony2012 in IEEE Magazine
    try:
        #ddx_com = -FBp["K_x"]*dx[0] - FBp["K_dx"]*dx[6]
        #ddy_com = -FBp["K_y"]*dx[1] - FBp["K_dy"]*dx[7]
        roll_com  = -.08 #+ ddx_com*FBp["sinYawSet"] - ddy_com*FBp["cosYawSet"]
        pitch_com = -.04 #+ ddx_com*FBp["cosYawSet"] + ddy_com*FBp["sinYawSet"]
        Delta_roll  = dx[3] - roll_com
        Delta_pitch = dx[4] - pitch_com
        lift = FBp["mg"] - FBp["K_z"]*dx[2] - FBp["K_dz"]*dx[8]
        tau1 = FBp["K_roll"]*Delta_roll + FBp["K_droll"]*dx[9]
        tau2 = FBp["K_pitch"]*Delta_pitch + FBp["K_dpitch"]*dx[10]
        tau3 = FBp["K_yaw"]*dx[5] + FBp["K_dyaw"]*dx[11]
        Tau = np.array([lift,tau1,tau2,tau3])
        wsquared = np.matmul(FBp["Gamma"],Tau)
        wsquared[wsquared < 0] = 0
        w = np.sqrt(wsquared)
        PW = ctrl.Speed2PW(w,v_battery,PWMp)
        PW = ctrl.RectifyControl(PW)
    except:
        ESC.StopMotors(mypi,pins)
        print("Feedback calculation error. Setting motors to zero and terminating.")
        sys.exit()
    return PW
