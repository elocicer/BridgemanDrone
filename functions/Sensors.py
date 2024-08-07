from Adafruit_BNO055 import BNO055
import functions.BNOSensor as BNO
import functions.Vicon as Vicon
import time
import functions.Controller as ctrl
import numpy as np

# CONNECT AND CALLIBRATE
def init(calibrate):
    # Raspberry Pi configuration with serial UART and RST connected to GPIO 18: UART mode must be turned on (PS1 pin = HIGH).
    # Connect to Voltage Divider
    i2c = busio.I2C(board.SCL,board.SDA)
    ads = ADS.ADS1115(i2c)
    VoltageDivider = AnalogIn(ads,ADS.P0)
    # Connect to IMU.
    bno = BNO055.BNO055(serial_port='/dev/serial0', rst=18)
    BNO.connectSensor(bno)
    print("Sensor connected!")
    if calibrate: # Optional
        BNO.callibrateSensor(bno)
    print("Sensor callibrated!\n\n")
    # Connect to Vicon
    vicon_client, mytracker = Vicon.connectVicon("192.168.0.101")
    object_name = "BridgemanDrone"
    return bno, mytracker, object_name, VoltageDivider

def getState(bno, mytracker, object_name, state, setpoint, cur_time, filter_states, Fp, yaw_looper, rawyaw_prev, mypi, pins, relay_pin, VoltageDivider,PWMp):
    rawx, rawy, rawz                                     = Vicon.GetLinearStates(mytracker, object_name, state[0:3])
    rawyaw, pitch, roll, droll, dpitch, dyaw, a_x, a_y, a_z = BNO.getStates(bno,mypi,pins,relay_pin)
    yaw, yaw_looper       = ctrl.RectifyYaw(rawyaw,rawyaw_prev,yaw_looper)
    prev_time = cur_time
    cur_time  = time.time()
    dt        = cur_time - prev_time
    #x, filter_states[0]       = ctrl.FilterSignal(rawx, dt, filter_states[0], Fp["Tx"], Fp["Kx"])
    #y, filter_states[1]       = ctrl.FilterSignal(rawy, dt, filter_states[1], Fp["Ty"], Fp["Ky"])
    #z, filter_states[2]       = ctrl.FilterSignal(rawz, dt, filter_states[2], Fp["Tz"], Fp["Kz"])
    x = rawx
    y = rawy
    z = rawz
    rawdxdt, rawdydt, rawdzdt = ctrl.EstimateRates(x, y, z, dt, state[0:3])
    #dxdt, filter_states[3]    = ctrl.FilterSignal(rawdxdt, dt, filter_states[3], Fp["Tdx"], Fp["Kdx"])
    #dydt, filter_states[4]    = ctrl.FilterSignal(rawdydt, dt, filter_states[4], Fp["Tdy"], Fp["Kdy"])
    #dzdt, filter_states[5]    = ctrl.FilterSignal(rawdzdt, dt, filter_states[5], Fp["Tdz"], Fp["Kdz"])
    dxdt = rawdxdt
    dydt = rawdydt
    dzdt = rawdzdt
    state = np.array([[x],[y],[z],[roll],[pitch],[yaw],[dxdt],[dydt],[dzdt],[droll],[dpitch],[dyaw]])
    dx = state - setpoint
    v_battery = VoltageDivider.voltage * PWMp["vDividerRatio"]
    return state, dx, v_battery, cur_time, filter_states, yaw_looper, rawyaw
