#OBU.py
#12/06/24
#Rémi Myard

#This is the code of the On Board Unit of the VACOP
#It takes inputs from the user and the can_bus, takes decisions and commands all actuators (propulsion, braking, steering)



##################################### Setup ###################################



import RPi.GPIO as GPIO
import can
import re
import time
import os
import tkinter as tk

#Set the DEVICE as On Board Unit (OBU). It will be used to sort incoming can messages destined to this DEVICE
DEVICE = "OBU"

# Set GPIO mode to BCM
GPIO.setmode(GPIO.BCM)

# Set GPIO pins for the propulsion controller
DIR_POSITIVE = 27 #make the car go forward
DIR_NEGATIVE = 23 #make the car go backward
SetTorquePin = 17 #set the current applied to the motors
CurrentLimitPin = 22 #set the current limit of the motors

# Set PWM frequency (Hz)
PWM_FREQ = 5000

# Pilot the motor
GPIO.setup(DIR_POSITIVE, GPIO.OUT)
GPIO.setup(DIR_NEGATIVE, GPIO.OUT)
GPIO.setup(SetTorquePin, GPIO.OUT)
GPIO.setup(CurrentLimitPin, GPIO.OUT)

# Set the initial direction
GPIO.output(DIR_POSITIVE, GPIO.HIGH) #for now we can only go forward
GPIO.output(DIR_NEGATIVE, GPIO.LOW)

# Initialize PWM for SetTorquePin
SetTorque = GPIO.PWM(SetTorquePin, PWM_FREQ)
CurrentLimit = GPIO.PWM(CurrentLimitPin, PWM_FREQ)

# Start PWM with 0% duty cycle
SetTorque.start(0)
CurrentLimit.start(0)

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
def can_send(device_id, order_id, data=None, ui=None):
    
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
        # Convert data back in decimal to display
        data = int.from_bytes(data_bytes, byteorder='big')

    # Create the CAN message
    can_message = can.Message(arbitration_id=arbitration_id, data=data_bytes, is_extended_id=False)
    
    # Convert data back in decimal to display
    data = int.from_bytes(data_bytes, byteorder='big')

    # Send the message on the CAN bus
    bus.send(can_message)
    message = f"sent: {device_id} {order_id} {data}"
    
    # Log to the terminal
    if ui:
        ui.log_to_terminal(message)

# Class to receive messages from the CAN bus
class CanReceive(can.Listener):
    def __init__(self, ui=None):
        super(CanReceive, self).__init__()
        self.last_received_message = None
        self.ui = ui  # Add a reference to the UserInterface instance
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
                # Log to the terminal
                if self.ui:
                    self.ui.log_to_terminal(message)
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
    SetTorque.ChangeDutyCycle(prop_set)

# Function to process can messages and user inputs
# Initialize variables
prop_override = 0
manual_prop_set = 0
last_steer_enable = 0
def processor(can_msg, user_msg, ui=None):
    global prop_override
    global manual_prop_set
    global last_steer_enable
    # Extract data from the user message 
    on_off_state = user_msg[0] #0=off, 1=on
    driving_mode = user_msg[1] #0=manual, 1=auto
    auto_brake_set = user_msg[2] #0=NO_BRAKE, 1=FULL_BRAKE
    auto_steer_set = user_msg[3] #0=FullLeft, 512=middle,1023=FullRight
    auto_prop_set = user_msg[4] #0=No prop #1023 = FULL prop

    # Extract data from the can message
    if can_msg is not None :
        order_id = can_msg[1] #order_id is the order we received
        data = can_msg[2] #data is the data attached to the order

    #Manage order_id
    if can_msg is not None:
        if order_id == "prop_override" and data == 1:
            prop_override = 1 #modify the global variable prop_override if we detect that the user is braking
            
        if order_id == "prop_override" and data == 0:
            prop_override = 0

        if order_id == "accel_pedal":
            manual_prop_set = data
        

    #Manual mode
    if driving_mode == 0:
        #reset braking
        if ui.last_braking_mode is None or ui.last_braking_mode != 0:
            can_send("BRAKE", "brake_set", 0, ui)
            ui.last_braking_mode = 0

        #steering
        if last_steer_enable is None or last_steer_enable != 0:
            can_send("BRAKE", "steer_enable", 0, ui)
            last_steer_enable = 0
        
        #propulsion
        if prop_override == 0:
            propulsion(manual_prop_set)

        else:
            propulsion(0)

    #Automatic mode
    else:
        #prop_override -> reset brake & propulsion=0
        if prop_override == 1:    
            #brake
            if ui.last_braking_mode is None or ui.last_braking_mode != 0:
                can_send("BRAKE", "brake_set", 0, ui)
                ui.last_braking_mode = 0
            #propulsion
            propulsion(0)
        else:
            #brake
            if ui.last_braking_mode is None or ui.last_braking_mode != auto_brake_set:
                if auto_brake_set == 1: #brake_set =1 means we want to brake
                    can_send("BRAKE", "brake_set", 1, ui)
                else:  #auto_brake_ser == 0 means we are not braking
                    can_send("BRAKE", "brake_set", 0, ui)
                ui.last_braking_mode = auto_brake_set
            
            #propulsion
            if auto_brake_set == 0: #if we are not braking we can accelerate
                propulsion(auto_prop_set)
            
            else: #if we are braking we stop the motor
                propulsion(0)

        
        #Steering
        if last_steer_enable is None or last_steer_enable != 1:
            can_send("BRAKE", "steer_enable", 1, ui)
            last_steer_enable = 1
            print(last_steer_enable)

        if ui.last_steering_value is None or ui.last_steering_value != auto_steer_set:
            #map value: 
            mapped_steering_value = int(((auto_steer_set+100)*1023)/200)
            can_send("STEER", "steer_pos_set", mapped_steering_value, ui)
            ui.last_steering_value = auto_steer_set


# Class to interact with the variables, it opens a control pannel where the car can be piloted. 
# From there we can switch to automatic/manual mode, we can steer, brake and accelerate. We can also see the feed of the can bus
class UserInterface:
    def __init__(self, root):
        self.root = root

        # Initialize variables
        self.on_off_state = 0  # Track the on/off state
        self.driving_mode = 0
        self.auto_brake_set = 0
        self.auto_steer_set = 0
        self.auto_prop_set = 0
        self.last_braking_mode = None  # Track the last braking mode state
        self.last_steering_value = None  # Track the last steering value

        # Create a frame to hold the controls
        frame_controls = tk.Frame(root)
        frame_controls.pack(side=tk.LEFT, pady=20, padx=20)

        # Create a frame to hold the label and button for start/stop
        frame_start_stop = tk.Frame(frame_controls)
        frame_start_stop.pack(pady=10)

        # Create a label for start/stop
        self.label_start_stop = tk.Label(frame_start_stop, text="Start/Stop", font=('Helvetica', 16))
        self.label_start_stop.pack(side=tk.LEFT)

        # Create a button for start/stop
        self.start_stop_button = tk.Button(frame_start_stop, text="off", command=self.toggle_on_off_state, font=('Helvetica', 16), width=10, height=3)
        self.start_stop_button.pack(side=tk.LEFT)

        # Create a frame to hold the label and button for driving mode
        frame_drive = tk.Frame(frame_controls)
        frame_drive.pack(pady=10)

        # Create a label for driving mode
        self.label_drive = tk.Label(frame_drive, text="Driving Mode", font=('Helvetica', 16))
        self.label_drive.pack(side=tk.LEFT)

        # Create a button for driving mode
        self.drive_button = tk.Button(frame_drive, text="manual", command=self.toggle_driving_mode, font=('Helvetica', 16), width=10, height=3)
        self.drive_button.pack(side=tk.LEFT)

        # Create a frame to hold the label and button for braking mode
        frame_brake = tk.Frame(frame_controls)
        frame_brake.pack(pady=10)

        # Create a label for braking mode
        self.label_brake = tk.Label(frame_brake, text="Braking Mode", font=('Helvetica', 16))
        self.label_brake.pack(side=tk.LEFT)

        # Create a button for braking mode
        self.brake_button = tk.Button(frame_brake, text="not braking", command=self.toggle_braking_mode, font=('Helvetica', 16), width=10, height=3)
        self.brake_button.pack(side=tk.LEFT)

        # Create a frame to hold the label and slider for steering value
        frame_steering = tk.Frame(frame_controls)
        frame_steering.pack(pady=10)

        # Create a label for steering value
        self.label_steering = tk.Label(frame_steering, text="Steering Value", font=('Helvetica', 16))
        self.label_steering.pack(side=tk.LEFT)

        # Create a scale (slider) for steering value
        self.steering_scale = tk.Scale(frame_steering, from_=-100, to=100, orient=tk.HORIZONTAL, length=300)
        self.steering_scale.set(0)  # Initial value at the center
        self.steering_scale.pack(side=tk.LEFT)

        # Create a frame to hold the label and slider for propulsion value
        frame_propulsion = tk.Frame(frame_controls)
        frame_propulsion.pack(pady=10)

        # Create a label for propulsion value
        self.label_propulsion = tk.Label(frame_propulsion, text="Propulsion Value", font=('Helvetica', 16))
        self.label_propulsion.pack(side=tk.LEFT)

        # Create a scale (slider) for propulsion value
        self.propulsion_scale = tk.Scale(frame_propulsion, from_=0, to=1023, orient=tk.HORIZONTAL, length=300)
        self.propulsion_scale.set(0)  # Initial value at the left
        self.propulsion_scale.pack(side=tk.LEFT)

        # Create a frame to hold the label and value for propulsion override
        frame_override = tk.Frame(frame_controls)
        frame_override.pack(pady=10)

        # Create a label for propulsion override
        self.label_override = tk.Label(frame_override, text="Propulsion Override:", font=('Helvetica', 16))
        self.label_override.pack(side=tk.LEFT)

        # Create a variable to display the propulsion override value
        self.label_override_value = tk.Label(frame_override, text="0", font=('Helvetica', 16))
        self.label_override_value.pack(side=tk.LEFT)

        # Create a text widget to act as a terminal
        self.terminal = tk.Text(root, height=20, width=50, font=('Helvetica', 12))
        self.terminal.pack(side=tk.RIGHT, padx=20, pady=20)

        # Bind event to slider release
        self.steering_scale.bind("<ButtonRelease-1>", self.update_steering_value)
        self.propulsion_scale.bind("<ButtonRelease-1>", self.update_propulsion_value)

    def toggle_on_off_state(self):
        self.on_off_state = 1 if self.on_off_state == 0 else 0
        self.start_stop_button.config(text="on" if self.on_off_state == 1 else "off")

    def toggle_driving_mode(self):
        self.driving_mode = 1 if self.driving_mode == 0 else 0
        self.drive_button.config(text="auto" if self.driving_mode == 1 else "manual")

    def toggle_braking_mode(self):
        self.auto_brake_set = 1 if self.auto_brake_set == 0 else 0
        self.brake_button.config(text="braking" if self.auto_brake_set == 1 else "not braking")

    def update_steering_value(self, event):
        self.auto_steer_set = int(self.steering_scale.get())

    def update_propulsion_value(self, event):
        self.auto_prop_set = int(self.propulsion_scale.get())

    def log_to_terminal(self, message):  # This is a terminal that logs the feed of the CAN bus
        self.terminal.insert(tk.END, message + '\n')
        self.terminal.see(tk.END)  # Auto-scroll to the end

    def ui_output_variables(self):
        return self.on_off_state, self.driving_mode, self.auto_brake_set, self.auto_steer_set, self.auto_prop_set

    def ui_input_variables(self, override_value):
        self.label_override_value.config(text=str(override_value))

def init(app, root, message_listener):
    print("waiting for start...\n")
    on_off_state = 0
    # We wait the user to start the VACOP
    while on_off_state == 0:
        root.update()
        user_msg = app.ui_output_variables() # wait for the start button to be pressed
        on_off_state = user_msg[0]
        if on_off_state == 1:
            print("initialization...\n")
            can_send("BRAKE", "start", 0, app) # When the VACOP is started, we send a start message on the can bus to initialise the actuators.
            can_send("STEER", "start", 0, app)
    print("waiting for devices to be ready...\n")
    ready = False
    # We wait the devices to reply the initialisation message
    while ready is False:
        root.update()
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
    can_send("BRAKE","brake_enable",None,app)
    can_send("STEER","steer_enable",None,app)
    print("Ready to use")
    return




#################################### MAIN ###################################################

def main():
    try:
        with can.interface.Bus(channel='can0', bustype='socketcan', receive_own_messages=False) as bus:
            
            
            # Initialize user interface
            root = tk.Tk()
            root.title("VACOP Control v1")
            app = UserInterface(root)

            # Start CAN bus
            message_listener = CanReceive(ui=app)
            can.Notifier(bus, [message_listener])

            while True:
                # Initialize VACOP
                init(app, root, message_listener)
                
                # Main operational loop
                running = True
                while running:
                    # Receive CAN message (from the devices: BRAKE or STEER)
                    can_msg = message_listener.can_input()
                    # Receive inputs from user 
                    root.update()
                    user_msg = app.ui_output_variables()

                    # Send outputs to user
                    app.ui_input_variables(prop_override)
                    
                    # Check if the on/off button has been pressed to stop the system
                    if user_msg[0] == 0:
                        print("System stopped.\n\n\n")
                        running = False
                        break
                    
                    # Process CAN messages and user messages, takes decisions and send orders to the devices
                    processor(can_msg, user_msg, ui=app)  # Pass the ui object here

    except KeyboardInterrupt:
        print("Exiting...")
        can_send("BRAKE", "stop")

    finally:
        bus.shutdown()
        GPIO.cleanup()

if __name__ == "__main__":
    main()
