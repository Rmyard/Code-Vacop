#use library time to generate steps
import RPi.GPIO as GPIO
import time

################################
# RPi and Motor Pre-allocations
################################
#
#define GPIO pins
GPIO.setmode(GPIO.BCM)
DIR_PIN= 17 # Direction (DIR) GPIO Pin
PUL_PIN = 26 # Step GPIO Pin
EN_PIN = 16 # Enable GPIO Pin
GPIO.setup([DIR_PIN, PUL_PIN, EN_PIN], GPIO.OUT)


###########################
# Actual motor control
###########################
#

def TurnCW():
    for _ in range(4000):
        GPIO.output(DIR_PIN, GPIO.HIGH)
        GPIO.output(PUL_PIN, GPIO.HIGH)
        time.sleep(0.001)
        GPIO.output(PUL_PIN, GPIO.LOW)


def TurnCCW():
   for _ in range(4000):
        GPIO.output(DIR_PIN, GPIO.LOW)
        GPIO.output(PUL_PIN, GPIO.HIGH)
        time.sleep(0.001)
        GPIO.output(PUL_PIN, GPIO.LOW)

GPIO.output(EN_PIN, GPIO.HIGH)

time1 = time.time()
TurnCW()
time2 = time.time()
print(time2-time1)


time1 = time.time()
TurnCCW()
time2 = time.time()
print(time2-time1)


GPIO.cleanup() # clear GPIO allocations after run
