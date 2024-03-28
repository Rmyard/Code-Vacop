#!/usr/bin/env python

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

class MyListener(can.Listener):
    
    def __init__(self):
        super(MyListener, self).__init__()
        self.last_received_message = None

    def on_message_received(self, msg):
        self.last_received_message = msg

    # Function to receive desired position from CAN bus
    def CAN_input(self):
        try:
            can_input = (self.last_received_message.data[0] << 8) + self.last_received_message.data[1]
            brake_pos_set = int(can_input)
            return brake_pos_set
        except AttributeError:
            print("No message received yet.")
            return None

# Function to read position from MCP3008
def Read_position():
    sensor_input = mcp.read_adc(0)
    brake_pos_real = int(sensor_input)
    return brake_pos_real

# Function to control PWM based on CAN input and position feedback
def Brake(brake_pos_set, brake_pos_real):
    try:
        # Calculate error
        error = brake_pos_set - brake_pos_real

        # Calculate control value
        control_value = Kp * error
        # Map the control value from -100 to 100 for pwm application
        control_value = (control_value/1023) * 100
        print("contr_val =",control_value)

        # Map control value to duty cycle
        if control_value > 0: # Extension
            print("1")
            extend_pwm.ChangeDutyCycle(control_value)
            retract_pwm.ChangeDutyCycle(0)
        elif control_value < 0: # Retraction
            print("2")
            extend_pwm.ChangeDutyCycle(0)
            retract_pwm.ChangeDutyCycle(-control_value)
        else: # Stop
            print("3")
            extend_pwm.ChangeDutyCycle(0)
            retract_pwm.ChangeDutyCycle(0)

    except ValueError:
        print("Invalid input. Please enter a valid integer.")
    


#bus = can.interface.Bus(channel='can0', bustype='socketcan')

def main():
    
    with can.Bus(channel='can0', bustype='socketcan', receive_own_messages=True) as bus:
        message_listener = MyListener()
        print_listener = can.Printer()
        can.Notifier(bus, [message_listener, print_listener])
        print(message_listener)

        while True:
            # Receive CAN message and position from sensors
            brake_pos_real = Read_position()
            brake_pos_set = message_listener.CAN_input()
            print("real = ",brake_pos_real)
            # Control brake based on desired and real positions
            if brake_pos_set != None:
                Brake(brake_pos_set, brake_pos_real)
                print("set = ",brake_pos_set)
                
           
            time.sleep(1)



if __name__ == "__main__":
    main()
