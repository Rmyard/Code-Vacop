#BRAKE.py
#03/06/24
#Rémi Myard

#This code is hosted by the BRAKE device, it is controling the braking actuator and is receiving data from the accelerator pedal and manual braking button. 
#It can also control the steering for now but at the end of the project the steering will be controlled by a different node.


##################################### Setup ###################################



# Import libraries
import RPi.GPIO as GPIO
import can
import os
import time
import Adafruit_MCP3008
import re

#Setup the bus
bus = can.interface.Bus(channel='can0', bustype='socketcan', receive_own_messages=False)

#Set the device as brake. It will setup the device id in the canbus
device = "BRAKE"

# Set GPIO mode to BCM
GPIO.setmode(GPIO.BCM)

# Set PWM frequency (Hz)
PWM_FREQ = 1000

# Set GPIO pins for actuator control
EXTEND_PWM_PIN = 17
RETRACT_PWM_PIN = 27
EXTEND_EN_PIN = 22
RETRACT_EN_PIN = 23
GPIO.setup([EXTEND_PWM_PIN, RETRACT_PWM_PIN, EXTEND_EN_PIN, RETRACT_EN_PIN], GPIO.OUT)

# Set GPIO pins for stepper control
DIR_PIN= 16 # Direction (DIR) GPIO Pin
PUL_PIN = 26 # Step GPIO Pin
EN_PIN = 6 # Enable GPIO Pin
GPIO.setup([DIR_PIN, PUL_PIN, EN_PIN], GPIO.OUT)
pulse = GPIO.PWM(PUL_PIN, PWM_FREQ)

# Create PWM instances
extend_pwm = GPIO.PWM(EXTEND_PWM_PIN, PWM_FREQ)
retract_pwm = GPIO.PWM(RETRACT_PWM_PIN, PWM_FREQ)

# Set GPIO pin for override brake button
BRAKE_PIN = 24
GPIO.setup(BRAKE_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

# MCP3008 configuration
CLK = 21
MISO = 19
MOSI = 20
CS = 7
mcp = Adafruit_MCP3008.MCP3008(clk=CLK, cs=CS, miso=MISO, mosi=MOSI)

# Proportional gain (Kp)
Kp = 2

#Limit the values of the braking actuator
MAX_EXTEND = 800
MIN_EXTEND = 300
THRESHOLD = 10

NO_BRAKE = 500
FULL_BRAKE = 600

#Limit the values of the steering actuator
LEFT_LIMIT = 10
RIGHT_LIMIT = 1013
NEUTRAL_POSITION = 512

############################### FUNCTIONS ###########################################



# Function to load the CAN_List.txt
def load_can_list(filename):
    device_id_map = {}
    device_id_reverse_map = {}
    order_id_map = {}
    order_id_reverse_map = {}

    with open(filename, 'r') as file:
        content = file.read()

    # Extract DeviceID mappings
    device_id_section = re.search(r'DeviceID:\s*{([^}]*)}', content)
    if device_id_section:
        device_lines = device_id_section.group(1).strip().split('\n')
        for line in device_lines:
            if '=' in line:
                key, value = line.split('=')
                key = key.strip()
                value = value.strip()
                device_id_map[key] = value
                device_id_reverse_map[value] = key

    # Extract OrderID mappings
    order_id_section = re.search(r'OrderID:\s*{([^}]*)}', content)
    if order_id_section:
        order_lines = order_id_section.group(1).strip().split('\n')
        for line in order_lines:
            if '=' in line:
                key, value = line.split('=')
                key = key.strip()
                value = value.strip()
                order_id_map[key] = value
                order_id_reverse_map[value] = key

    return device_id_map, order_id_map, device_id_reverse_map, order_id_reverse_map

# Load CAN list mappings
device_id_map, order_id_map, device_id_reverse_map, order_id_reverse_map = load_can_list('CAN_List.txt')

# Function to send message on the CAN bus
def can_send(bus, device_ID, order_ID, data=None):
    # Convert human-readable IDs to their corresponding hex values
    device_value = device_id_map.get(device_ID)
    order_value = order_id_map.get(order_ID)

    if device_value is None or order_value is None:
        raise ValueError("Invalid device_id or order_id")
    
    # Create a CAN message with device_value followed by order_value
    arbitration_id = int(device_value + order_value, 16)

    # Convert the data into a list of bytes
    data_bytes = []
    if data is not None:
        while data > 0:
            data_bytes.insert(0, data & 0xFF)
            data >>= 8
        # Convert data back in decimal to display
        data = int.from_bytes(data_bytes, byteorder='big')

    # Create the CAN message
    can_message = can.Message(arbitration_id=arbitration_id, data=data_bytes, is_extended_id=False)
    
    # Convert data back in decimal to display
    data = int.from_bytes(data_bytes, byteorder='big')

    # Send the message on the CAN bus
    bus.send(can_message)
    print("sent:", device_ID, order_ID, data)

# Function to receive messages from the CAN bus
class CanReceive(can.Listener):
    def __init__(self):
        super (CanReceive, self).__init__()
        self.last_received_message = None

    def on_message_received(self, msg):
        self.last_received_message = msg

    # Function to receive desired position from CAN bus
    def can_input(self):
        # Assuming self.last_received_message is the last received CAN message
        if self.last_received_message is None:
            return None  # Return None if no message has been received

        # Extract device_ID and order_ID from the arbitration ID
        device_value = hex(self.last_received_message.arbitration_id >> 8)[2:].zfill(2)
        order_value = hex(self.last_received_message.arbitration_id & 0xFF)[2:].zfill(2)

        # Extract data from the CAN message
        data = int.from_bytes(self.last_received_message.data, byteorder='big')

        # Convert hex values to human-readable strings
        device_ID = device_id_reverse_map.get(device_value, device_value)
        order_ID = order_id_reverse_map.get(order_value, order_value)

        if device_ID == device or device_ID == "STEER": #pour l'instant STEER = BRAKE, NE PAS OUBLIER DENLEVER LE OR
            
            print("received:", device_ID, order_ID, data)
            # Return device_ID, order_ID, and data in a tuple
            return device_ID, order_ID, data

# Function to read brake position from MCP3008
def read_brake_position():
    brake_pos_real = int(mcp.read_adc(1))
    return brake_pos_real

# Function to read steer position from MCP3008
def read_steer_position():
    steer_pos_real = int(mcp.read_adc(2)) #third pin of the mcp on the test bench
    print("steer_pos_real = ",steer_pos_real)
    return steer_pos_real

# Function to read acceleration from MCP3008
last_accel_pedal = None
def accel():
    global last_accel_pedal
    
    # Read the current acceleration value from MCP3008
    accel_pedal = int(mcp.read_adc(0))
    
    # Limit the value
    if accel_pedal < 170:
        accel_pedal = 170
    elif accel_pedal > 875:
        accel_pedal = 875

    # Map the value
    accel_pedal = int(((accel_pedal - 170) / (875 - 170)) * 1023)

    # Check if the new value is different from the last value
    if accel_pedal != last_accel_pedal:
        # Update the last value
        last_accel_pedal = accel_pedal
        can_send(bus,"OBU","accel_pedal",accel_pedal)

# Function to read the manual braking button and send an override message on the can bus
DEBOUNCE_TIME = 1 #time period during which subsequent interrupts will be ignored
last_callback_time = 0 #initialise the last callback time for the button
button_state = 0 #initialise the button state to 0
def brake_override(BRAKE_PIN):
    global last_callback_time
    global button_state
    current_time = int(time.time() * 1000)  # Convert current time to milliseconds

    if current_time - last_callback_time > DEBOUNCE_TIME:
        last_callback_time = current_time
        button_state = 1 if button_state == 0 else 0
        can_send(bus,"OBU","brake_override",button_state)

# Function to control the braking motor
def brake(brake_pos_set):
    try:
        #Read actuator position
        brake_pos_real= read_brake_position()
        print("brake_pos_real = ",brake_pos_real)
        print("brake_pos_set = ",brake_pos_set)

        #Limit the set value
        if brake_pos_set < MIN_EXTEND:
            brake_pos_set = MIN_EXTEND
        
        if brake_pos_set > MAX_EXTEND:
            brake_pos_set = MAX_EXTEND

        #Check the real value, deactivate movement if necessary. This is to ensure that the actuator stays in the safe limits.
        if brake_pos_real < MIN_EXTEND:
            GPIO.output(RETRACT_EN_PIN, GPIO.LOW)
            print("min extend reached")

        else:
            GPIO.output(RETRACT_EN_PIN, GPIO.HIGH)

        if brake_pos_real > MAX_EXTEND:
            GPIO.output(EXTEND_EN_PIN, GPIO.LOW)
            print("max extend reached")
        
        else:
            GPIO.output(EXTEND_EN_PIN, GPIO.HIGH)
        
        # Calculate error (setpoint-real_value)
        error = brake_pos_set - brake_pos_real

        # Calculate control value
        control_value = Kp * error
        print("ctr_val = ",control_value)
        
        # Map the control value from -100 to 100 for pwm application
        control_value = int((control_value/1023) * 100)
        if control_value < -100: #limiter
            control_value = -100
        if control_value > 100:
            control_value = 100
        if -THRESHOLD<control_value<THRESHOLD:#deadzone (avoid activating the actuator with the noise)
            control_value = 0

        # Map control value to duty cycle (this essentially controls the motor speed and direction)
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

# Function to control the steering motor
def steer(steer_pos_set):
    steer_pos_real = read_steer_position()
    error = steer_pos_set - steer_pos_real
        
    #safety mechanism
    if steer_pos_real < LEFT_LIMIT:
        GPIO.output(EN_PIN, GPIO.HIGH) #Desactivate the motor in case of reaching min value
        print("steer min value reached, motor blocked")
    elif steer_pos_real > RIGHT_LIMIT: #Desactivate the motor in case of reaching max value
        GPIO.output(EN_PIN, GPIO.HIGH)
        print("steer max value reached, motor blocked")
    else:
        GPIO.output(EN_PIN, GPIO.LOW)

    #Define direction of spin
    if error > THRESHOLD:
        GPIO.output(DIR_PIN, GPIO.HIGH)
        #send pulse
        pulse.ChangeDutyCycle(50)

    elif error < -THRESHOLD:
        GPIO.output(DIR_PIN, GPIO.LOW)
        #send pulse
        pulse.ChangeDutyCycle(50)

    else:
        GPIO.output(EN_PIN, GPIO.HIGH)
    
    #Check real value
    steer_pos_real = read_steer_position()
    error = steer_pos_set - steer_pos_real

#Function to process incoming can messages and take decisions
def processor(can_msg):
    global button_state
    if can_msg is not None:
        #extract data from the can message
        order_id = can_msg[1]
        data = can_msg[2]
        
        #braking
        if order_id == "brake_set" and data == 0:#can message with no brake
            brake(NO_BRAKE)
        elif order_id == "brake_set" and data == 1:#can message with full brake
            brake(FULL_BRAKE)
        elif order_id == "brake_set" and data != 0 or data != 1: #error in can message
            brake(NO_BRAKE)
            print("error")
        elif button_state == 1: #user is braking, we retract the actuator
            brake(NO_BRAKE)
            print("brake override detected")

        #steering
        if order_id == "steer_pos_set":
            steer(data)

# Function to initialise the actuator
def init():
    print("position initialization...\n")
    brake_pos_real = read_brake_position()
    while brake_pos_real != NO_BRAKE:
        brake_pos_real = read_brake_position()
        brake(NO_BRAKE)
        print("position = ",brake_pos_real)
        time.sleep(0.1)
    print("position initialized")



#################################### MAIN #########################################################



def main():
    try:
        with can.interface.Bus(channel='can0', bustype='socketcan', receive_own_messages=False) as bus:
            # Start CAN bus
            message_listener = CanReceive()
            can.Notifier(bus, [message_listener])

            # Start PWM
            extend_pwm.start(0)
            retract_pwm.start(0)
            pulse.start(0)

            #Start button event detection
            GPIO.add_event_detect(BRAKE_PIN, GPIO.BOTH, callback=brake_override)

            #Initialise de position of the actuator
            init()
            
            while True:
                # Receive CAN message
                can_msg = message_listener.can_input()
                # Process the can message 
                processor(can_msg)
                # Read the value from the accelerator pedal and send the acceleration value on the can bus
                accel()
            
                time.sleep(0.1)

    
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
