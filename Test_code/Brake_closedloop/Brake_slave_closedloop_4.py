#Brake_slave_closedloop.py
#04/04/24
#Rémi Myard
#This is a very basic code to control the brake actuator.
#It has the 3 main fuctions to ensure control loop:
#Read_position
#CAN_receive
#Brake
#This code is implementing the accelerator pedal and manual braking detection

#arbitration_id
#000 OBU
#001 Brakes
#002 Steering
#003 Motors

# Import libraries
import RPi.GPIO as GPIO
import can
import os
import time
import Adafruit_MCP3008
from collections import deque

# Set GPIO mode to BCM
GPIO.setmode(GPIO.BCM)

# Set GPIO pins
EXTEND_PWM_PIN = 17
RETRACT_PWM_PIN = 27
EXTEND_EN_PIN = 22
RETRACT_EN_PIN = 23
GPIO.setup(EXTEND_PWM_PIN, GPIO.OUT)
GPIO.setup(RETRACT_PWM_PIN, GPIO.OUT)
GPIO.setup(EXTEND_EN_PIN, GPIO.OUT)
GPIO.setup(RETRACT_EN_PIN, GPIO.OUT)

# Setup the manual brake button
BRAKE_PIN = 24
GPIO.setup(BRAKE_PIN, GPIO.IN)
DEBOUNCE_TIME = 200 #time period during which subsequent interrupts will be ignored
last_callback_time = 0 #initialise the last callback time for the button

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
Kp = 10

#Limit values of position
max_extend = 800
min_extend = 300
threshold = 10
no_brake = 400
full_brake = 600

# Function to receive CAN messages
class can_receive(can.Listener):
    
    def __init__(self):
        super(can_receive, self).__init__()
        self.last_received_message = None

    def on_message_received(self, msg):
        self.last_received_message = msg

    # Function to receive desired position from CAN bus
    def can_input(self):
        try:
            can_input = (self.last_received_message.data[0] << 8) + self.last_received_message.data[1]
            brake_pos_set = int(can_input)
            
            return brake_pos_set
        except AttributeError:
            print("No message received yet.")
            return None


# Function to read position from MCP3008
def Read_position():
    brake_pos_real = int(mcp.read_adc(1))
    return brake_pos_real

# Function to read acceleration from MCP3008
def Read_accel():
    accel_pedal = int(mcp.read_adc(0))
    
    #limit the value
    if accel_pedal < 170:
        accel_pedal = 170

    if accel_pedal > 875:
        accel_pedal = 875

    #map the value
    accel_pedal = int(((accel_pedal - 170) / (875 - 170))*1023)

    return accel_pedal

# Function to read the manual braking button
def Read_button(BRAKE_PIN):
    global last_callback_time
    current_time = int(time.time() * 1000)  # Convert current time to milliseconds
    if current_time - last_callback_time > DEBOUNCE_TIME:
        print("Edge detected")
    last_callback_time = current_time

# Function to control PWM based on CAN input and position feedback
def Brake(brake_pos_set, brake_pos_real):
    try:
        
        # Calculate error
        error = brake_pos_set - brake_pos_real
        print("error = ", error)

        # Calculate control value
        control_value = Kp * error 
        
        # Map the control value from -100 to 100 for pwm application
        control_value = int((control_value/1023) * 100)
        if control_value < -100: #limiter
            control_value = -100
        if control_value > 100:
            control_value = 100
        if -threshold<control_value<threshold:#deadzone (couper acctionnement autour du bruit)
            control_value = 0

        print("contr_val =",control_value)
        print("\n")

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

def secu(brake_pos_set, brake_pos_real):
    #Limit the set value
    if brake_pos_set < min_extend:
        brake_pos_set = min_extend
    
    if brake_pos_set > max_extend:
        brake_pos_set = max_extend

    #Check the real value, deactivate movement if necessary
    if brake_pos_real < min_extend:
        GPIO.output(RETRACT_EN_PIN, GPIO.LOW)

    else:
        GPIO.output(RETRACT_EN_PIN, GPIO.HIGH)

    if brake_pos_real > max_extend:
        GPIO.output(EXTEND_EN_PIN, GPIO.LOW)
    
    else:
        GPIO.output(EXTEND_EN_PIN, GPIO.HIGH)

def init():
    brake_pos_real = Read_position()
    while brake_pos_real != init_value:
        brake_pos_set = init_value

def main():
    
    try:
        with can.Bus(channel='can0', bustype='socketcan', receive_own_messages=True) as bus:
            # Start CAN bus
            message_listener = can_receive()
            can.Notifier(bus, [message_listener])
            print(message_listener)
            # Start PWM
            extend_pwm.start(0)
            retract_pwm.start(0)
            #Start button event detection
            GPIO.add_event_detect(BRAKE_PIN, GPIO.RISING, callback=Read_button)
            #initialise position
            
            while True:
                # Receive CAN message
                brake_pos_set = message_listener.can_input()
                
                # Receive position and acceleration
                accel_pedal = Read_accel()
                brake_pos_real = Read_position()
                print("real = ",brake_pos_real)
                print("accel = ",accel_pedal)

            
                # Control brake based on desired and real positions
                if brake_pos_set != None:
                    # Check if everything is in order
                    secu(brake_pos_set, brake_pos_real)
                    # Execute the brake function
                    Brake(brake_pos_set, brake_pos_real)
                    print("set = ",brake_pos_set)
                time.sleep(0.05)
            
                # Send CAN message
                CAN_Send(accel_pedal,0x001)

    
    except KeyboardInterrupt:
        # Clean up GPIO
        extend_pwm.stop()
        retract_pwm.stop()
        GPIO.cleanup()

    finally:
        # Close the CAN bus
        bus.shutdown()


if __name__ == "__main__":
    main()
