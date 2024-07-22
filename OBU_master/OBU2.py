#OBU.py
#10/07/24
#Rémi Myard

#This is the code of the On Board Unit of the VACOP
#It takes inputs from the user and the can_bus, takes decisions and commands all actuators (propulsion, braking, steering)



##################################### Setup ###################################



import RPi.GPIO as GPIO
import can
import re
import time
import Adafruit_MCP3008
import pygame

#Set the DEVICE as On Board Unit (OBU). It will be used to sort incoming can messages destined to this DEVICE
DEVICE = "OBU"

# Set GPIO mode to BCM
GPIO.setmode(GPIO.BCM)

# Set PWM frequency (Hz)
PWM_FREQ_PROP = 5000

# Define the dead zone
DEAD_ZONE = 0.10

# Initialize joystick input

IS_VACOP_ON = False # state of the vacop

INPUT_MODE = 0 # 0 = joystick, 1 controllers
INPUT_MODE_CHANGED = False

PROPULSION = 0 # Propulsion value
PROPULSION_CHANGED = False

DIRECTION = 512 # Direction value
DIRECTION_CHANGED = False

BREAK_STATE = 1 # X button (0 = pressed = not break, 1 = released = break)
BREAK_STATE_CHANGED = False

# Initialize variables
prop_override = 0 # state of the braking sensor (1 = activated, 0 else) 
manual_prop_set = 0 # value of the accelerating pedal (0 <= manual_prop_set <= 1023) 
# last_steer_enable = 0 # steer state (0 = user can steer (MANUAL MODE) , 1 = user can't steer, the motor is bloked(AUTO MODE))



# Set GPIO pins for the propulsion controller
DIR_1_PIN = 23 #chose the direction of the motor1 (HIGH = CCW) (LOW = CW)
DIR_2_PIN = 17 #chose the direction of the motor2 (HIGH = CCW) (LOW = CW)
SET_TORQUE_1_PIN = 24 #set the current applied to motor 1
SET_TORQUE_2_PIN = 22 #set the current applied to motor 2
STO1_PIN = 16 #Set Torque Off motor 1
STO2_PIN = 26 #Set Torque Off motor 2

GPIO.setup(DIR_1_PIN, GPIO.OUT)
GPIO.setup(DIR_2_PIN, GPIO.OUT)
GPIO.setup(SET_TORQUE_1_PIN, GPIO.OUT)
GPIO.setup(SET_TORQUE_2_PIN, GPIO.OUT)
GPIO.setup(STO1_PIN, GPIO.OUT)
GPIO.setup(STO2_PIN, GPIO.OUT)

GPIO.output(DIR_1_PIN, GPIO.HIGH) #for now we can only go forward
GPIO.output(DIR_2_PIN, GPIO.LOW) #for now we can only go forward

# Initialize PWM pins
SetTorque1 = GPIO.PWM(SET_TORQUE_1_PIN, PWM_FREQ_PROP)
SetTorque2 = GPIO.PWM(SET_TORQUE_2_PIN, PWM_FREQ_PROP)

# Start PWM instances
SetTorque1.start(0)
SetTorque2.start(0)


#Setup the can bus
bus = can.interface.Bus(channel='can0', bustype='socketcan', receive_own_messages=False)


############################### FUNCTIONS ###########################################


# Function to load the CAN_List.txt 
# CAN_List.txt is a file containing DEVICE adresses and a list of orders. This file is basically an encryption DEVICE to read and write can messages. It can be easily modified and copy/pasted across all devices.
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
def can_send(device_id, order_id, data=None):
    
    # Convert human-readable IDs to their corresponding hex values
    device_value = device_id_map.get(device_id)
    order_value = order_id_map.get(order_id)

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
            print("data=",data)
            print("data_bytes=",data_bytes)
        # Convert data back in decimal to display
        data = int.from_bytes(data_bytes, byteorder='big')

    # Create the CAN message
    can_message = can.Message(arbitration_id=arbitration_id, data=data_bytes, is_extended_id=False)
    
    # Convert data back in decimal to display
    data = int.from_bytes(data_bytes, byteorder='big')

    # Send the message on the CAN bus
    bus.send(can_message)
    message = f"sent: {device_id} {order_id} {data}"

# Class to receive messages from the CAN bus
class CanReceive(can.Listener):
    def __init__(self):
        super(CanReceive, self).__init__()
        self.last_received_message = None
        self.last_data = {}  # Dictionary to store the last data for each device-order

    def on_message_received(self, msg):
        self.last_received_message = msg

    # Function to receive desired position from CAN bus
    def can_input(self):
        # Assuming self.last_received_message is the last received CAN message
        if self.last_received_message is None:
            return None  # Return None if no message has been received

        # Extract device_id and order_id from the arbitration ID
        device_value = hex(self.last_received_message.arbitration_id >> 8)[2:].zfill(2)
        order_value = hex(self.last_received_message.arbitration_id & 0xFF)[2:].zfill(2)

        # Extract data from the CAN message
        data = int.from_bytes(self.last_received_message.data, byteorder='big')

        # Convert hex values to human-readable strings
        device_id = device_id_reverse_map.get(device_value, device_value)
        order_id = order_id_reverse_map.get(order_value, order_value)

        if device_id == DEVICE:  # only process messages destined to this DEVICE
            key = (device_id, order_id)
            if key not in self.last_data or self.last_data[key] != data:
                message = f"received: {device_id} {order_id} {data}"
                # Update the last data
                self.last_data[key] = data
            # Return device_ID, order_id, and data in a tuple
            return device_id, order_id, data


# Function to control the motors
# The motors are at the back of the car, with the OBU. The OBU controlls the motors with GPIO, no need for the can bus.
def propulsion(prop_set):
    # Map the value
    prop_set = int((prop_set * 100) / 1023)
    # Set the duty cycle of the motors
    SetTorque1.ChangeDutyCycle(prop_set)
    SetTorque2.ChangeDutyCycle(prop_set)

def propulsion_enable(enable):
    if enable is True :
        GPIO.output(STO1_PIN, GPIO.HIGH)
        GPIO.output(STO2_PIN, GPIO.HIGH)
    else:
        GPIO.output(STO1_PIN, GPIO.LOW)
        GPIO.output(STO2_PIN, GPIO.LOW)

# Function to process can messages and user inputs

def processor(can_msg):
    global prop_override
    global manual_prop_set
    # global last_steer_enable
    global PROPULSION
    global PROPULSION_CHANGED
    global DIRECTION
    global DIRECTION_CHANGED
    global INPUT_MODE
    global INPUT_MODE_CHANGED
    global BREAK_STATE
    global BREAK_STATE_CHANGED

    # Extract data from the can message
    if can_msg is not None :
        order_id = can_msg[1] #order_id is the order we received
        data = can_msg[2] #data is the data attached to the order

    # Manage order_id
    if can_msg is not None:
        if order_id == "prop_override" and data == 1:
            prop_override = 1 #modify the global variable prop_override if we detect that the user is braking
            
        if order_id == "prop_override" and data == 0:
            prop_override = 0

        if order_id == "accel_pedal":
            manual_prop_set = data

    # Manual mode
    if INPUT_MODE == 1:
        #reset braking >>>>>>>>>>>> TODO with controller
        if INPUT_MODE_CHANGED:
            can_send("BRAKE", "brake_set", 0)
            can_send ("BRAKE","brake_set", 0)
            can_send("STEER", "steer_enable", 0) # shutdown steer motor
            INPUT_MODE_CHANGED = False
            print("mode manu")
        
        #if user isn't braking, enable accelerator
        if prop_override == 0:
            propulsion(manual_prop_set)
        else:
            propulsion(0) # desable accelerator

    # Automatic mode
    else:
        
        if INPUT_MODE_CHANGED:
            can_send("STEER", "steer_enable", 1) # enable steer motor
            print("mode auto")
            INPUT_MODE_CHANGED = False
        
        #Brake
        if BREAK_STATE_CHANGED:
            if BREAK_STATE == 1 : #X = False means we want to brake
                can_send("BRAKE", "brake_set", 1)
            else:  #X == True means we are not braking
                can_send("BRAKE", "brake_set", 0)
            BREAK_STATE_CHANGED = False
        
        #Propulsion
        if BREAK_STATE == 0:
            if PROPULSION_CHANGED:
                propulsion(PROPULSION)
                PROPULSION_CHANGED = False
        else:
            propulsion(0)
        
        # update steer position
        if DIRECTION_CHANGED:
            can_send("STEER", "steer_pos_set", DIRECTION)
            DIRECTION_CHANGED = False

########################### Joystick functions ###############################
# Function to create a dead zone and avoid joystick "noise"
def apply_dead_zone(value, dead_zone):
    if abs(value) < dead_zone:
        return 0
    return value

# Function to map the input values
def normalize_value(value, min_value, max_value):
    # Normalize from [-1, 1] to [0, 1023]
    return int((value + 1) * (max_value - min_value) / 2 + min_value)


def get_joystick_input() : 
    global IS_VACOP_ON
    global PROPULSION
    global PROPULSION_CHANGED
    global DIRECTION
    global DIRECTION_CHANGED
    global INPUT_MODE
    global INPUT_MODE_CHANGED
    global BREAK_STATE
    global BREAK_STATE_CHANGED

    for event in pygame.event.get():      
        if event.type == pygame.QUIT:
            running = False
        # PRESS BUTTON X (button 0) to move forward
        elif ((event.type == pygame.JOYBUTTONDOWN) and (event.button == 0)):
            if(BREAK_STATE != 0):
                print(f"Button {event.button} (X) pressed")
                BREAK_STATE = 0
                BREAK_STATE_CHANGED = True
                
        # RELEASED BUTTON X (button 0) to activate break  
        elif ((event.type == pygame.JOYBUTTONUP) and (event.button == 0)):
            if(BREAK_STATE != 1):
                print(f"Button {event.button} (X) released")
                BREAK_STATE = 1
                BREAK_STATE_CHANGED = True

        # PRESS BUTTON SQUARE (button 2) for joystick input
        elif ((event.type == pygame.JOYBUTTONDOWN) and (event.button == 2)):
            print(f"Button {event.button} (SQUARE) pressed")
            INPUT_MODE = 0
            print("mode Joystick")
            INPUT_MODE_CHANGED = True
  
        # PRESS BUTTON TRIANGLE (button 3) for controllers input
        elif ((event.type == pygame.JOYBUTTONDOWN) and (event.button == 3)):
            print(f"Button {event.button} (TRIANGLE) pressed")
            print("Mode manual")
            INPUT_MODE = 1
            INPUT_MODE_CHANGED = True

        # TRIGGER ACCELERATION (axis 5)
        elif ((event.type == pygame.JOYAXISMOTION) and (event.axis == 5)):
            axis_value = event.value
            normalized_value = normalize_value(axis_value, 0, 1023)
            PROPULSION = normalized_value
            PROPULSION_CHANGED = True
            print(f"ACCELERATION moved to {normalized_value}")

        # JOYSTICK FOR DIRECTION (axis 0)
        elif ((event.type == pygame.JOYAXISMOTION) and (event.axis == 0)):
            axis_value = apply_dead_zone(event.value, DEAD_ZONE)
            normalized_value = normalize_value(axis_value, 0, 1023)
            DIRECTION = normalized_value
            DIRECTION_CHANGED = True
            print(f"DIRECTION  moved to {normalized_value}")
        
        # PRESS BUTTON START (button 11)
        elif ((event.type == pygame.JOYBUTTONDOWN) and (event.button == 1)):
            print(f"Button {event.button} (START) pressed")
            IS_VACOP_ON = True

############################### INITIALIZATION ################################


def init(message_listener):
    global IS_VACOP_ON
    print("waiting for start...\n")
    
    # We wait the user to start the VACOP
    while not IS_VACOP_ON:
        get_joystick_input()
    
    print("initialization...\n")
    can_send("BRAKE", "start", 0) # When the VACOP is started, we send a start message on the can bus to initialise the actuators.
    can_send("STEER", "start", 0)
    print("waiting for devices to be ready...\n")
    ready = False
    # We wait the devices to reply the initialisation message
    while ready is False:
        can_msg = message_listener.can_input()
        if can_msg is not None:
            # Extract data from the can message
            order_id = can_msg[1] # order_id is the order we received
            data = can_msg[2] # data is the data attached to the order
            if order_id == "brake_rdy": # we wait for the actuators response
                ready = True
            if order_id == "brake_not_rdy":
                ready = False
                print("One of the devices encountered a problem")

    # We enable movement of the actuators
    propulsion(0)
    propulsion_enable(True)#activate propulsion
    can_send("BRAKE","brake_enable",None)
    can_send("STEER","steer_enable",None)
    print("Ready to use")

    return

#################################### MAIN ###################################################

def main():
    try:
        with can.interface.Bus(channel='can0', bustype='socketcan', receive_own_messages=False) as bus:

            # Start CAN bus
            message_listener = CanReceive()
            can.Notifier(bus, [message_listener])

            # Initialize Pygame and the joystick module
            pygame.init()
            pygame.joystick.init()

            # Initialize the first joystick
            joystick = None
            if pygame.joystick.get_count() > 0:
                joystick = pygame.joystick.Joystick(0)
                joystick.init()
                print(f"Joystick initialized: {joystick.get_name()}")
            else:
                print("No joystick found.")
                pygame.quit()
                exit -1

            # Initialize VACOP
            init(message_listener)
            
            # Main operational loop
            running = True
            while running:
                # Receive CAN message (from the devices: BRAKE or STEER)
                can_msg = message_listener.can_input()
                # Receive inputs from user 
                get_joystick_input() #Update joystick global variables (DIRECTION, PROPULSION, X , MODE)
                    
                # Process CAN messages and user messages, takes decisions and send orders to the devices
                processor(can_msg)  # Pass the ui object here

    except KeyboardInterrupt:
        print("Exiting...")

    finally:
        bus.shutdown()
        GPIO.cleanup()

if __name__ == "__main__":
    main()
