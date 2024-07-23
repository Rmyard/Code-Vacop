import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)

# Set GPIO pin for override brake button
BRAKE_PIN = 24
GPIO.setup(BRAKE_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

# Function to read the manual braking button and send an override message on the can bus

button_state = 0

def brake_override(BRAKE_PIN):
    global button_state
    if GPIO.input(BRAKE_PIN) == GPIO.HIGH:
        button_state = 1
        print("button_state = ", button_state)
    else:
        button_state = 0
        print("button_state = ", button_state)

GPIO.add_event_detect(BRAKE_PIN, GPIO.BOTH, callback=brake_override, bouncetime=100)

while True:
    pass

