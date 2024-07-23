#motor to test the brushless motors (input from 0 to 100 % of the torque of the motor)

import RPi.GPIO as GPIO
import time

# Set GPIO mode to BCM
GPIO.setmode(GPIO.BCM)

# Set GPIO pins
DIR1 = 23
DIR2 = 17
SetTorquePin1 = 24
SetTorquePin2 = 22
STO1 = 16
STO2 = 26

# Set PWM frequency (Hz)
PWM_FREQ = 5000

# Pilot the motor
GPIO.setup(DIR1, GPIO.OUT)
GPIO.setup(DIR2, GPIO.OUT)

GPIO.setup(SetTorquePin1, GPIO.OUT)
GPIO.setup(SetTorquePin2, GPIO.OUT)

GPIO.setup(STO1,GPIO.OUT)
GPIO.setup(STO2,GPIO.OUT)


# Set the initial direction
GPIO.output(DIR1, GPIO.LOW)
GPIO.output(DIR2, GPIO.HIGH)

GPIO.output(STO1, GPIO.HIGH)
GPIO.output(STO2, GPIO.HIGH)


# Initialize PWM for SetTorquePin
SetTorque1 = GPIO.PWM(SetTorquePin1, PWM_FREQ)
SetTorque2 = GPIO.PWM(SetTorquePin2, PWM_FREQ)



# Start PWM with 0% duty cycle
SetTorque1.start(0)
SetTorque2.start(0)


try:
    while True:
        # Get user input for SetTorque
        user_input = float(input("Enter SetTorque value (or q to quit): "))
        
        if user_input == 'q':
            break  # Exit the loop if 'q' is entered
        
        # Set the duty cycle based on user input
        SetTorque1.ChangeDutyCycle(user_input)
        SetTorque2.ChangeDutyCycle(user_input)


except KeyboardInterrupt:
    pass  # Handle keyboard interrupt (Ctrl+C)

finally:
    # Clean up GPIO pins
    GPIO.cleanup()
