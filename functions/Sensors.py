from Adafruit_BNO055 import BNO055
import functions.BNOSensor as BNO
import functions.Vicon as Vicon
import time
import functions.Controller as ctrl
import numpy as np
import smbus

# CONNECT AND CALLIBRATE
def init(calibrate):
    # Raspberry Pi configuration with serial UART and RST connected to GPIO 18: UART mode must be turned on (PS1 pin = HIGH).
    # Connect to Voltage Divider
    #i2c = busio.I2C(board.SCL,board.SDA)
    #ads = ADS.ADS1115(i2c)
    #VoltageDivider = AnalogIn(ads,ADS.P0)

    # Configure ADS1115 on I2C bus
    config_register = 0x01
    config = 0x4083 # Set range to +/-6.144V, samples per second to 860, differential comparison between AIN0 and AIN1, and continuous mode
    ADS1115 = {"bus": smbus.SMBus(1), "address": 0x48, "conversion_register": 0x00, "vDividerRatio": 2.6366}
    ADS1115["bus"].write_i2c_block_data(ADS1115["address"], config_register, [(config >> 8) & 0xFF, config & 0xFF])
    
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
    return bno, mytracker, object_name, ADS1115

def getState(bno, mytracker, object_name, state, setpoint, cur_time, filter_states, Fp, yaw_looper, rawyaw_prev, mypi, pins, relay_pin, ADS1115):
    # Get Vicon States
    rawx, rawy, rawz                                     = Vicon.GetLinearStates(mytracker, object_name, state[0:3])
    # Mark time
    prev_time = cur_time
    cur_time  = time.time()
    dt        = cur_time - prev_time
    # Get IMU states
    rawyaw, rawpitch, rawroll, rawdroll, rawdpitch, rawdyaw, a_x, a_y, a_z = BNO.getStates(bno,mypi,pins,relay_pin)
    # Rectify Yaw beteen 0 and 2pi
    yaw, yaw_looper       = ctrl.RectifyYaw(rawyaw,rawyaw_prev,yaw_looper)
    # Estimate linear rates
    rawdxdt, rawdydt, rawdzdt = ctrl.EstimateRates(rawx, rawy, rawz, dt, state[0:3])
    # Apply filters
    x, filter_states[0]     = ctrl.FilterSignal(rawx, dt, filter_states[0], Fp["Tx"], Fp["Kx"])
    y, filter_states[1]     = ctrl.FilterSignal(rawy, dt, filter_states[1], Fp["Ty"], Fp["Ky"])
    z, filter_states[2]     = ctrl.FilterSignal(rawz, dt, filter_states[2], Fp["Tz"], Fp["Kz"])
    #x = rawx
    #y = rawy
    #z = rawz
    roll,  filter_states[3] = ctrl.FilterSignal(rawroll,  dt, filter_states[3], Fp["Troll"],  Fp["Kroll"])
    pitch, filter_states[4] = ctrl.FilterSignal(rawpitch, dt, filter_states[4], Fp["Tpitch"], Fp["Kpitch"])
    yaw,   filter_states[5] = ctrl.FilterSignal(yaw,   dt, filter_states[5], Fp["Tyaw"],   Fp["Kyaw"])
    #roll  = rawroll
    #pitch = rawpitch
    dxdt, filter_states[6]    = ctrl.FilterSignal(rawdxdt, dt, filter_states[6], Fp["Tdx"], Fp["Kdx"])
    dydt, filter_states[7]    = ctrl.FilterSignal(rawdydt, dt, filter_states[7], Fp["Tdy"], Fp["Kdy"])
    dzdt, filter_states[8]    = ctrl.FilterSignal(rawdzdt, dt, filter_states[8], Fp["Tdz"], Fp["Kdz"])
    #dxdt = rawdxdt
    #dydt = rawdydt
    #dzdt = rawdzdt  
    droll,  filter_states[9]  = ctrl.FilterSignal(rawdroll,  dt, filter_states[9], Fp["Tdroll"], Fp["Kdroll"])
    dpitch, filter_states[10] = ctrl.FilterSignal(rawdpitch, dt, filter_states[10], Fp["Tdpitch"], Fp["Kdpitch"])
    dyaw,   filter_states[11] = ctrl.FilterSignal(rawdyaw,   dt, filter_states[11], Fp["Tdyaw"], Fp["Kdyaw"])
    #droll  = rawdroll
    #dpitch = rawdpitch
    #dyaw   = rawdyaw
    # Read battery voltage
    data = ADS1115["bus"].read_i2c_block_data(ADS1115["address"],ADS1115["conversion_register"],2)
    raw_adc = data[0] << 8 | data[1] # read the first 16 bits and concatinate them
    if raw_adc > 32767: #max value of 2^15 bits
         raw_adc -= 65536 #max value of 2^16 bits
    v_channel = raw_adc * 6.144 / 32768.0 # bits to volts conversion
    v_battery = v_channel * ADS1115["vDividerRatio"] # convert voltage divider voltage to battery voltage
    # Compile states as array
    state = np.array([[x],[y],[z],[roll],[pitch],[yaw],[dxdt],[dydt],[dzdt],[droll],[dpitch],[dyaw]])
    dx = state - setpoint
    return state, dx, v_battery, cur_time, filter_states, yaw_looper, rawyaw
