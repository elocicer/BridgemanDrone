import RPi.GPIO as GPIO

motorPin1 = 18
motorPin2 = 37
motorPin3 = 11
motorPin4 = 36
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
GPIO.setup(motorPin1,GPIO.OUT)
GPIO.setup(motorPin2,GPIO.OUT)
GPIO.setup(motorPin3,GPIO.OUT)
GPIO.setup(motorPin4,GPIO.OUT)
m1 = GPIO.PWM(motorPin1, 50)
m2 = GPIO.PWM(motorPin2, 50)
m3 = GPIO.PWM(motorPin3, 50)
m4 = GPIO.PWM(motorPin4, 50)
input("Check that battery is disconnected from the base, then press enter.")
m1.start(4)
m2.start(4)
m3.start(4)
m4.start(4)
input("Connect the battery, then press enter.")
print("4 beeps indicates drone is armed. Otherwise, needs callibration.")
print("End arming sequence.")
m1.ChangeDutyCycle(5.5)
m2.ChangeDutyCycle(5.5)
m3.ChangeDutyCycle(5.5)
m4.ChangeDutyCycle(5.5)