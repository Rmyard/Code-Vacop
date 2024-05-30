#OBU.py
#30/05/24
#Rémi Myard



##################################### Setup ###################################



import RPi.GPIO as GPIO
import can
import re
import time
import os
import tkinter as tk

#Set the device as brake. It will setup the device id in the canbus
device = "OBU"

# Set GPIO mode to BCM
GPIO.setmode(GPIO.BCM)

# Set GPIO pins
DIR_POSITIVE = 27
DIR_NEGATIVE = 23
SetTorquePin = 17
CurrentLimitPin = 22

# Set PWM frequency (Hz)
PWM_FREQ = 5000

# Pilot the motor
GPIO.setup(DIR_POSITIVE, GPIO.OUT)
GPIO.setup(DIR_NEGATIVE, GPIO.OUT)
GPIO.setup(SetTorquePin, GPIO.OUT)
GPIO.setup(CurrentLimitPin, GPIO.OUT)

# Set the initial direction
GPIO.output(DIR_POSITIVE, GPIO.HIGH)
GPIO.output(DIR_NEGATIVE, GPIO.LOW)

# Initialize PWM for SetTorquePin
SetTorque = GPIO.PWM(SetTorquePin, PWM_FREQ)
CurrentLimit = GPIO.PWM(CurrentLimitPin, PWM_FREQ)

# Start PWM with 0% duty cycle
SetTorque.start(0)
CurrentLimit.start(0)

bus = can.interface.Bus(channel='can0', bustype='socketcan', receive_own_messages=False)


############################### FUNCTIONS ###########################################



# Function to load the CAN_List.txt
def LoadCanList(filename):
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
device_id_map, order_id_map, device_id_reverse_map, order_id_reverse_map = LoadCanList('CAN_List.txt')

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
        # Convert data back in decimal to display
        data = int.from_bytes(data_bytes, byteorder='big')

    # Create the CAN message
    can_message = can.Message(arbitration_id=arbitration_id, data=data_bytes, is_extended_id=False)
    
    # Convert data back in decimal to display
    data = int.from_bytes(data_bytes, byteorder='big')

    # Send the message on the CAN bus
    bus.send(can_message)
    print("sent:", device_id, order_id, data)

# Class to receive messages from the CAN bus
class can_receive(can.Listener):
    def __init__(self):
        super(can_receive, self).__init__()
        self.last_received_message = None

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

        if device_id == device: #only process messages destined to this device
            
            print("received:", device_id, order_id, data)
            # Return device_ID, order_id, and data in a tuple
            return device_id, order_id, data

# Function to control the motors
def propulsion(prop_value):
    # Map the value
    prop_value = int((prop_value * 100) / 1023)
    # Set the duty cycle of the motors
    SetTorque.ChangeDutyCycle(prop_value)

# Function to process can messages and user inputs
def processor(can_msg, user_msg):
    global propulsion_override
    #extract data from the user message 
    driving_mode = user_msg[0]
    braking_mode = user_msg[1]
    steering_value = user_msg[2]
    prop_value = user_msg[3]

    if driving_mode == 1 and braking_mode == 1:
        can_send("BRAKE","brake_set",1)
        
    if braking_mode == 0:
        can_send("BRAKE","brake_set",0)

    if driving_mode == 1 and propulsion_override == 0:
            propulsion(prop_value)
            
    if propulsion_override == 1:
            propulsion(0)


    if can_msg is not None:
        #extract data from the can message
        order_id = can_msg[1]
        data = can_msg[2]
        
        
        if order_id == "brake_override" and data == 1:
            propulsion_override = 1

        if order_id == "brake_override" and data == 0:
            propulsion_override = 0

        #manual mode
        if driving_mode == 0 and order_id == "accel_pedal" and propulsion_override == 0:
            propulsion(data)


# Class to interact with the variables
class UserInterface:
    def __init__(self, root):
        self.root = root

        # Initialize variables
        self.driving_mode = 0
        self.braking_mode = 0
        self.steering_value = 0
        self.prop_value = 0

        # Create a frame to hold the controls
        frame_controls = tk.Frame(root)
        frame_controls.pack(side=tk.LEFT, pady=20, padx=20)

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

        # Create a text widget to act as a terminal
        self.terminal = tk.Text(root, height=20, width=50, font=('Helvetica', 12))
        self.terminal.pack(side=tk.RIGHT, padx=20, pady=20)

        # Bind event to slider release
        self.steering_scale.bind("<ButtonRelease-1>", self.update_steering_value)
        self.propulsion_scale.bind("<ButtonRelease-1>", self.update_propulsion_value)

    def toggle_driving_mode(self):
        self.driving_mode = 1 if self.driving_mode == 0 else 0
        self.drive_button.config(text="auto" if self.driving_mode == 1 else "manual")
        self.log_to_terminal(f"Driving Mode: {'auto' if self.driving_mode == 1 else 'manual'}")

    def toggle_braking_mode(self):
        self.braking_mode = 1 if self.braking_mode == 0 else 0
        self.brake_button.config(text="braking" if self.braking_mode == 1 else "not braking")
        self.log_to_terminal(f"Braking Mode: {'braking' if self.braking_mode == 1 else 'not braking'}")

    def update_steering_value(self, event):
        self.steering_value = int(self.steering_scale.get())
        self.log_to_terminal(f"Steering Value: {self.steering_value}")

    def update_propulsion_value(self, event):
        self.prop_value = int(self.propulsion_scale.get())
        self.log_to_terminal(f"Propulsion Value: {self.prop_value}")

    def log_to_terminal(self, message):
        self.terminal.insert(tk.END, message + '\n')
        self.terminal.see(tk.END)  # Auto-scroll to the end

    def get_values(self):
        return self.driving_mode, self.braking_mode, self.steering_value, self.prop_value



#################################### MAIN ###################################################



def main():
    global propulsion_override
    try:
        with can.interface.Bus(channel='can0', bustype='socketcan', receive_own_messages=False) as bus:
            # Start CAN bus
            message_listener = can_receive()
            can.Notifier(bus, [message_listener])
            # Initialise variables
            propulsion_override = 0
            root = tk.Tk()
            root.title("Braking Mode and Steering Control")
            app = UserInterface(root)
            
            while True:
                # Receive CAN message
                can_msg = message_listener.can_input()
                # Receive inputs from user
                root.update()
                user_msg = app.get_values()
                # Process CAN messages and user messages
                processor(can_msg, user_msg)
                time.sleep(0.1)

                
                

    except KeyboardInterrupt:
        print("Exiting...")

    finally:
        bus.shutdown()
        GPIO.cleanup()

if __name__ == "__main__":
    main()
