#use library RpiMotorLib
import RPi.GPIO as GPIO
from RpiMotorLib import RpiMotorLib
import time

################################
# RPi and Motor Pre-allocations
################################
#
#define GPIO pins
DIR_PIN= 17 # Direction (DIR) GPIO Pin
PUL_PIN = 12 # Step GPIO Pin
EN_PIN = 16 # Enable GPIO Pin

# Declare a instance of class pass GPIO pins numbers and the motor type 
turn_cw = RpiMotorLib.A4988Nema(DIR_PIN, PUL_PIN, (-1,-1,-1), "DRV8825")
turn_ccw = RpiMotorLib.A4988Nema(DIR_PIN, PUL_PIN, (-1,-1,-1), "DRV8825")
GPIO.setup([DIR_PIN, PUL_PIN, EN_PIN], GPIO.OUT)


###########################
# Actual motor control
###########################
#

def TurnCW():
    turn_cw.motor_go(True, # True=Clockwise, False=Counter-Clockwise
                        "Full" , # Step type (Full,Half,1/4,1/8,1/16,1/32)
                        10000, # number of steps
                        .0003, # PUL_PIN delay [sec]
                        False, # True = print verbose output 
                        .05) # initial delay [sec]

def TurnCCW():
    turn_ccw.motor_go(False, # True=Clockwise, False=Counter-Clockwise
                        "Full" , # Step type (Full,Half,1/4,1/8,1/16,1/32)
                        10000, # number of steps
                        .0003, # PUL_MINUS_PIN delay [sec]
                        False, # True = print verbose output 
                        .05) # initial delay [sec]



time1 = time.time()
TurnCW()
time2 = time.time()
print(time2-time1)


time1 = time.time()
TurnCCW()
time2 = time.time()
print(time2-time1)


GPIO.cleanup() # clear GPIO allocations after run
