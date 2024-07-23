#Brake_withCAN_slave_openloop
#Pilot the motor controler with a pwm signal
#Message received from can0
#madeby:Rémi Myard

import RPi.GPIO as GPIO
import can
import os


# Set GPIO mode to BCM
GPIO.setmode(GPIO.BCM)

# Set GPIO pins for PWM
FORWARD_PWM_PIN = 17
BACKWARD_PWM_PIN = 27
FORWARD_EN_PIN = 22
BACKWARD_EN_PIN = 23
GPIO.setup(FORWARD_PWM_PIN, GPIO.OUT)
GPIO.setup(BACKWARD_PWM_PIN, GPIO.OUT)
GPIO.setup(FORWARD_EN_PIN, GPIO.OUT)
GPIO.setup(BACKWARD_EN_PIN, GPIO.OUT)

# Set PWM frequency (Hz)
PWM_FREQ = 100

# Create PWM instances
forward_pwm = GPIO.PWM(FORWARD_PWM_PIN, PWM_FREQ)
backward_pwm = GPIO.PWM(BACKWARD_PWM_PIN, PWM_FREQ)

#Enable pins :

GPIO.output(FORWARD_EN_PIN, GPIO.HIGH)
GPIO.output(BACKWARD_EN_PIN, GPIO.HIGH)

# Function to control PWM based on CAN bus input
def control_pwm(can_input):
    try:
        # Convert CAN input to integer
        value = int(can_input)

        # Map CAN input to duty cycle (0-100)
        
        duty_cycle = abs(value - 128)  # 128 corresponds to stop position
        
        if duty_cycle > 100:
            duty_cycle = 100  # Limit duty cycle to 100%
        print(duty_cycle)
        # Set PWM duty cycle for forward and backward directions
        if value > 128:  # Extension
            forward_pwm.ChangeDutyCycle(duty_cycle)
            backward_pwm.ChangeDutyCycle(0)
        elif value < 128:  # Retraction
            forward_pwm.ChangeDutyCycle(0)
            backward_pwm.ChangeDutyCycle(duty_cycle)
        else:  # Stop
            forward_pwm.ChangeDutyCycle(0)
            backward_pwm.ChangeDutyCycle(0)

    except ValueError:
        print("Invalid input. Please enter a valid integer.")

# Initialize CAN bus interface
#os.system(sudo ip link set can0 up type can bitrate 1000000)
bus = can.interface.Bus(channel='can0', bustype='socketcan')

try:
    # Start PWM
    forward_pwm.start(0)
    backward_pwm.start(0)

    print("Listening for CAN messages...")

    # Continuous listening for CAN messages
    while True:
        # Receive CAN message
        message = bus.recv()

        # Extract value from CAN message (assuming it's the first byte)
        can_input = message.data[0]

        # Control PWM based on CAN input
        control_pwm(can_input)

except KeyboardInterrupt:
    # Clean up GPIO
    forward_pwm.stop()
    backward_pwm.stop()
    GPIO.cleanup()

finally:
    # Close the CAN bus
    bus.shutdown()
