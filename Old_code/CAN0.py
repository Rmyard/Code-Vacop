#CAN0.py
#simulate a CAN node
#used to develop the can_send, CanReceive and load_can_list functions

import can
import re
import time

device_ID = "OBU"

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

        # Extract device_ID and order_ID from the arbitration ID
        device_value = hex(self.last_received_message.arbitration_id >> 8)[2:].zfill(2)
        order_value = hex(self.last_received_message.arbitration_id & 0xFF)[2:].zfill(2)

        # Extract data from the CAN message
        data = int.from_bytes(self.last_received_message.data, byteorder='big')

        # Convert hex values to human-readable strings
        device_ID = device_id_reverse_map.get(device_value, device_value)
        order_ID = order_id_reverse_map.get(order_value, order_value)

        print("received:", device_ID, order_ID, data)

        # Return device_ID, order_ID, and data in a tuple
        return device_ID, order_ID, data

def main():
    try:
        with can.interface.Bus(channel='can0', bustype='socketcan', receive_own_messages=False) as bus:
            # Start CAN bus
            message_listener = can_receive()
            notifier = can.Notifier(bus, [message_listener])
            
            while True:
                # Receive CAN message
                can_msg = message_listener.can_input()
                
                # Send CAN message
                can_send(bus, "BRAKE", "start")
                time.sleep(0.5)
                can_send(bus, "STEER", "start")
                time.sleep(0.5)

    except KeyboardInterrupt:
        print("Exiting...")

    finally:
        bus.shutdown()

if __name__ == "__main__":
    main()
