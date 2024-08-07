#BRAKE.py
#18/07/24
#Rémi Myard

#This code is hosted by the BRAKE DEVICE, it is controling the braking actuator and is receiving data from the accelerator pedal and manual braking button. 
#It can also control the steering for now but at the end of the project the steering will be controlled by a different node.



##################################### Setup ###################################



# Import libraries
import RPi.GPIO as GPIO
import can
import time
import Adafruit_MCP3008
import re
import csv


#Setup the bus
bus = can.interface.Bus(channel='can0', bustype='socketcan', receive_own_messages=False)

#Set the DEVICE as brake. It will setup the DEVICE id in the canbus
DEVICE = "BRAKE"

# Set GPIO mode to BCM
GPIO.setmode(GPIO.BCM)


# Set PWM frequency (Hz)
PWM_FREQ_STEER = 1000
PWM_FREQ_BRAKE = 20000

# Set GPIO pins for actuator control
EXTEND_PWM_PIN = 13
RETRACT_PWM_PIN = 12
EXTEND_EN_PIN = 23
RETRACT_EN_PIN = 24
GPIO.setup([EXTEND_PWM_PIN, RETRACT_PWM_PIN, EXTEND_EN_PIN, RETRACT_EN_PIN], GPIO.OUT)
extend_pwm = GPIO.PWM(EXTEND_PWM_PIN, PWM_FREQ_BRAKE)
retract_pwm = GPIO.PWM(RETRACT_PWM_PIN, PWM_FREQ_BRAKE)

# Set GPIO pins for stepper control
STEER_DIR_PIN= 16 # Direction GPIO Pin
STEER_PUL_PIN = 26 # Pulse GPIO Pin
STEER_EN_PIN = 6 # Enable GPIO Pin
GPIO.setup([STEER_EN_PIN, STEER_PUL_PIN, STEER_DIR_PIN], GPIO.OUT)
pulse = GPIO.PWM(STEER_PUL_PIN, PWM_FREQ_STEER)

# Set GPIO pin for override brake button
BRAKE_PIN = 25
GPIO.setup(BRAKE_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

# MCP3008 configuration
CLK = 21
MISO = 19
MOSI = 20
CS = 7
mcp = Adafruit_MCP3008.MCP3008(clk=CLK, cs=CS, miso=MISO, mosi=MOSI)

# Proportional gain (KP_BRAKE)
KP_BRAKE = 10
KP_STEER = 1

#Setup values of the braking actuator
BRAKE_MAX_EXTEND = 923
BRAKE_MIN_EXTEND = 100
BRAKE_THRESHOLD = 5

NO_BRAKE = 650
FULL_BRAKE = 800

#Setup values of the steering actuator
STEER_LEFT_LIMIT = 100
STEER_RIGHT_LIMIT = 923
NEUTRAL_POSITION = 512

STEER_THRESHOLD = 10



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
def can_send(device_ID, order_ID, data=None):
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
    #print("sent:", device_ID, order_ID, data)

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

        if device_ID == DEVICE or device_ID == "STEER": #pour l'instant STEER = BRAKE, NE PAS OUBLIER DENLEVER LE OR
            
            #print("received:", device_ID, order_ID, data)
            # Return device_ID, order_ID, and data in a tuple
            return device_ID, order_ID, data

# Function to read brake position from MCP3008
def read_brake_position():
    brake_pos_real = int(mcp.read_adc(1))
    return brake_pos_real

# Function to read steer position from MCP3008
def read_steer_position():
    steer_pos_real = int(mcp.read_adc(2)) #third pin of the mcp on the test bench
    return steer_pos_real

# Function to read acceleration from MCP3008
last_accel_pedal = None
ACCEL_THRESHOLD = 0
def read_accelerator():
    global last_accel_pedal
    
    # Read the current acceleration value from MCP3008
    accel_pedal = int(mcp.read_adc(0))
    
    # Limit the value
    if accel_pedal < 250: #ancien = 170
        accel_pedal = 250
    elif accel_pedal > 875:
        accel_pedal = 875

    # Map the value
    accel_pedal = int(((accel_pedal - 250) / (875 - 250)) * 1023)

    # Check if the new value differs from the last value by more than the threshold
    if last_accel_pedal is None or abs(accel_pedal - last_accel_pedal) > ACCEL_THRESHOLD:
        # Update the last value
        last_accel_pedal = accel_pedal
        can_send("OBU","accel_pedal",accel_pedal)

# Function to read the manual braking button and send an override message on the can bus
button_state = 0
def brake_override(BRAKE_PIN):
    global button_state

    if GPIO.input(BRAKE_PIN) == GPIO.HIGH:
        button_state = 1
        can_send("OBU", "prop_override", button_state)
    else:
        button_state = 0
        can_send("OBU", "prop_override", button_state)

# Open the CSV file in write mode to clear it and write the header
with open('sensor_log.csv', 'w', newline='') as csvfile:
    csvwriter = csv.writer(csvfile)
    csvwriter.writerow(['Time', 'Accel Pedal', 'Brake Pos Set', 'Brake Pos Real', 'Steer Pos Set', 'Steer Pos Real'])
def log_sensor(brake_pos_set, steer_pos_set, start_time):
    brake_pos_real = read_brake_position()
    steer_pos_real = read_steer_position()
    accel_pedal = read_accelerator()
    elapsed_time = time.time() - start_time

    # Append the sensor data to the CSV file
    with open('sensor_log.csv', 'a', newline='') as csvfile:
        csvwriter = csv.writer(csvfile)
        csvwriter.writerow([elapsed_time, accel_pedal, brake_pos_set, brake_pos_real, steer_pos_set, steer_pos_real])

# Function to control the braking motor
def brake(brake_pos_set, enable):
    try:
        if enable == True:
     
            #Read actuator position
            brake_pos_real = read_brake_position()
            #print("brake_real = ", brake_pos_real)
            #print("brake_set = ", brake_pos_set)
            
            #Check if brake_pos_real is between MAX and MIN
            #This is to ensure that the actuator stays in the safe limits.
            if BRAKE_MIN_EXTEND < brake_pos_real < BRAKE_MAX_EXTEND:
                
                GPIO.output(EXTEND_EN_PIN, GPIO.HIGH)
                GPIO.output(RETRACT_EN_PIN, GPIO.HIGH)
            
                # Calculate error (setpoint-real_value)
                error = brake_pos_set - brake_pos_real

                # Calculate control value
                control_value = KP_BRAKE * error
                
                # Map the control value from -100 to 100 for pwm application
                control_value = int((control_value/1023) * 100)
                if control_value < -100: #limiter
                    control_value = -100
                if control_value > 100:
                    control_value = 100
                if -BRAKE_THRESHOLD<control_value<BRAKE_THRESHOLD:#deadzone (avoid activating the actuator with the noise)
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

            else :
                extend_pwm.ChangeDutyCycle(0)
                retract_pwm.ChangeDutyCycle(0)
                GPIO.output(RETRACT_EN_PIN, GPIO.LOW)
                GPIO.output(EXTEND_EN_PIN, GPIO.LOW)
                print("min or max extend reached")


        else:
            #deactivate movement of the motor
            extend_pwm.ChangeDutyCycle(0)
            retract_pwm.ChangeDutyCycle(0)
            GPIO.output(EXTEND_EN_PIN, GPIO.LOW)
            GPIO.output(RETRACT_EN_PIN, GPIO.LOW)

    except ValueError:
        print("Invalid input. Please enter a valid integer.")

# Function to control the steering motor
def steer(steer_pos_set, enable):
    if enable == True:
        #Enable the motor
        GPIO.output(STEER_EN_PIN, GPIO.LOW)
        #read real position
        steer_pos_real = read_steer_position()
        #calculate error
        error = steer_pos_set - steer_pos_real
        #calculate control value
        control_value = KP_STEER * error

        #safety mechanism
        if steer_pos_real < STEER_LEFT_LIMIT: #Desactivate the motor in case of reaching min value
            GPIO.output(STEER_EN_PIN, GPIO.HIGH) 
            print("steer min value reached, motor blocked")
        elif steer_pos_real > STEER_RIGHT_LIMIT: #Desactivate the motor in case of reaching max value
            GPIO.output(STEER_EN_PIN, GPIO.HIGH)
            print("steer max value reached, motor blocked")
        else: #Activate the motor
            GPIO.output(STEER_EN_PIN, GPIO.LOW)

        #Define direction of spin and send PWM pulses
        if control_value > STEER_THRESHOLD: 
            GPIO.output(STEER_DIR_PIN, GPIO.HIGH) #change direction
            pulse.ChangeDutyCycle(50) #send pulse

        elif control_value < -STEER_THRESHOLD:
            GPIO.output(STEER_DIR_PIN, GPIO.LOW) #change direction
            pulse.ChangeDutyCycle(50) #send pulse

        else:
            pulse.ChangeDutyCycle(0)
    
    else:
        #disable the motor
        pulse.ChangeDutyCycle(0)
        GPIO.output(STEER_EN_PIN, GPIO.HIGH)

#Function to log the sensor values
def log_sensor(brake_pos_set, steer_pos_set, start_time):
    brake_pos_real = read_brake_position()
    steer_pos_real = read_steer_position()
    accel_pedal = read_accelerator()
    elapsed_time = time.time() - start_time

    # Append the sensor data to the CSV file
    with open('sensor_log.csv', 'a', newline='') as csvfile:
        csvwriter = csv.writer(csvfile)
        csvwriter.writerow([elapsed_time, accel_pedal, brake_pos_set, brake_pos_real, steer_pos_set, steer_pos_real])

#Function to process incoming can messages and take decisions
#initialize last orders
last_brake = NO_BRAKE
last_steer = NEUTRAL_POSITION
steer_enable = 1 #steer is enabled for initialization
running = False
def processor(can_msg):
    global last_brake
    global last_steer
    global steer_enable
    global running
    global start_time
    
    #extract data from the can message
    if can_msg is not None:
        
        order_id = can_msg[1]
        data = can_msg[2]
        
        #Brake
        if order_id == "brake_set" and data == 0:#can message with no brake
            last_brake = NO_BRAKE

        if order_id == "brake_set" and data == 1:#can message with full brake
            last_brake = FULL_BRAKE
       
        #steering
        if order_id == "steer_pos_set":
            last_steer = data

        if order_id == "steer_enable":
            steer_enable = data

        if order_id == "stop":
            running = False

        if order_id == "start":
            running = True

    # Using the memory last_brake to pilot the braking actuator
    actual_brake = read_brake_position()
    if  actual_brake != last_brake:
        brake(last_brake, True)

    
# Using the memory last_steer to control the steering actuator
    """
    actual_steer = read_steer_position()
    if actual_steer != last_steer:
        if steer_enable == 0: 
            steer(last_steer, False) #motor disabled
        else:
            steer(last_steer, True) #motor enabled
    """
    log_sensor(last_brake, last_steer, start_time)
        

# Function to initialise the actuators
def init(message_listener):
    global running
    print("waiting for starting order...\n")
    #We wait for the OBU to send the start message
    while running is False:
        can_msg = message_listener.can_input()
        if can_msg is not None:
            # Extract data from the can message
            order_id = can_msg[1] #order_id is the order we received
            data = can_msg[2] #data is the data attached to the order
            if order_id == "start":
                running = True
    #Once the start message is received we start to initialise the actuators
    print("braking initialization...\n")
    brake_pos_real = read_brake_position()
    while brake_pos_real != NO_BRAKE:
        brake(NO_BRAKE, True)
        brake_pos_real = read_brake_position()
        
    
    #Once the braking actuator is done, we initialize the steering
    print("steering initialization...\n")
    steer_pos_real = read_steer_position()
    """
    while steer_pos_real != NEUTRAL_POSITION:
       steer(NEUTRAL_POSITION, True)
       brake(NO_BRAKE, True) #We still update the brake during this phase.
       steer_pos_real = read_steer_position()
    """
    can_send("OBU","brake_rdy")
    can_send("OBU","brake_rdy")
    can_send("OBU","brake_rdy")
    
    print("actuators initialized")
    



#################################### MAIN #########################################################



def main():
    global running
    global start_time
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
            GPIO.add_event_detect(BRAKE_PIN, GPIO.BOTH, callback=brake_override, bouncetime=20)

            while True:
                #Initialise de position of the actuator
                init(message_listener)
                print("System running.\n\n\n")
                start_time = time.time()#get time for the sensor_log file
                running = True
                while running:
                    # Receive CAN message
                    can_msg = message_listener.can_input()
                    
                    # Process the can message 
                    processor(can_msg)
                    # Read the value from the accelerator pedal and send the acceleration value on the can bus
                    read_accelerator()
                
                # When the actuator has been stopped by the OBU
                print("System stopped.\n\n\n")
                brake(NO_BRAKE,False) #deactivate braking
                steer(last_steer,False) #deactivate steering
                    

    
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
     
