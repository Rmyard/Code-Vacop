#Brake_withCAN_slave_closedloop.py
#12/03/24
#Rémi Myard
#This is a very basic code to control the brake actuator.
#It has the 3 main fuctions to ensure control loop:
#Read_position
#CAN_receive
#Brake
#Problem : it is stuck on the "bus.recv()"

#arbitration_id
#000 OBU
#001 Brakes
#002 Steering
#003 Motors

import RPi.GPIO as GPIO
import can
import os
import time
import Adafruit_GPIO.SPI as SPI
import Adafruit_MCP3008
import matplotlib.pyplot as plt

# Set GPIO mode to BCM
GPIO.setmode(GPIO.BCM)

# Set GPIO pins for PWM
EXTEND_PWM_PIN = 17
RETRACT_PWM_PIN = 27
GPIO.setup(EXTEND_PWM_PIN, GPIO.OUT)
GPIO.setup(RETRACT_PWM_PIN, GPIO.OUT)

# Set PWM frequency (Hz)
PWM_FREQ = 100

# Create PWM instances
extend_pwm = GPIO.PWM(EXTEND_PWM_PIN, PWM_FREQ)
retract_pwm = GPIO.PWM(RETRACT_PWM_PIN, PWM_FREQ)

# MCP3008 configuration
CLK = 21
MISO = 19
MOSI = 20
CS = 7
mcp = Adafruit_MCP3008.MCP3008(clk=CLK, cs=CS, miso=MISO, mosi=MOSI)

# Proportional gain (Kp)
Kp = 1

# Function to read position from MCP3008
def Read_position():
    sensor_input = mcp.read_adc(0)
    brake_pos_real = int(sensor_input)
    return brake_pos_real

# Function to receive desired position from CAN bus
def CAN_Receive():
    message = bus.recv()
    can_input = (message.data[0] << 8) + message.data[1]
    brake_pos_set = int(can_input)
    return brake_pos_set

# Function to control PWM based on CAN input and position feedback
def Brake(brake_pos_set, brake_pos_real):
    try:
        # Calculate error
        error = brake_pos_set - brake_pos_real

        # Calculate control value
        control_value = Kp * error
        # Map the control value from -100 to 100 for pwm application
        control_value = (control_value/1023) * 100

        print("set = ",brake_pos_set,"  type = ",type(brake_pos_set))
        print("real = ",brake_pos_real, "  type = ",type(brake_pos_set))
        print("contr_val =",control_value,"type =",type(control_value))

        # Map control value to duty cycle
        if control_value > 0: # Extension
            extend_pwm.ChangeDutyCycle(control_value)
            retract_pwm.ChangeDutyCycle(0)
        elif control_value < 0: # Retraction
            extend_pwm.ChangeDutyCycle(0)
            retract_pwm.ChangeDutyCycle(-control_value)
        else: # Stop
            extend_pwm.ChangeDutyCycle(0)
            retract_pwm.ChangeDutyCycle(0)

    except ValueError:
        print("Invalid input. Please enter a valid integer.")

# Initialize CAN bus interface
time.sleep(0.1)
bus = can.interface.Bus(channel='can0', bustype='socketcan')

try:
    # Start PWM
    extend_pwm.start(0)
    retract_pwm.start(0)

    print("Listening for CAN messages...")

    # Continuous listening for CAN messages
    while True:
        # Receive CAN message and position from sensors
        print("1")
        brake_pos_real = Read_position()
        print("2")
        brake_pos_set = CAN_Receive()
        print("3")
        # Control brake based on desired and real positions
        Brake(brake_pos_set, brake_pos_real)

        print("real =",brake_pos_real)

    

except KeyboardInterrupt:
    # Clean up GPIO
    extend_pwm.stop()
    retract_pwm.stop()
    GPIO.cleanup()

finally:
    # Close the CAN bus
    bus.shutdown()
