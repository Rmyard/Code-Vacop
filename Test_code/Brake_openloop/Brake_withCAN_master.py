#Brake_withCAN_master
#Takes a user input and send it on the can bus can0
#dont forget to bring can up with "sudo ip link set can0 up type can bitrate 1000000"
#madeby:Rémi Myard

import can
import os

# Initialize CAN bus interface
#os.system(sudo ip link set can0 up type can bitrate 1000000)
bus = can.interface.Bus(channel='can0', bustype='socketcan')

# Function to map user input to CAN signal
def map_to_can(user_input):
    try:
        # Convert input to integer
        value = int(user_input)

        # Map input to CAN signal (0-255)
        if -100 <= value <= 0:  # Retracting
            can_signal = int((value + 100) * 128 / 100)
        elif 0 <= value <= 100:  # Extension
            can_signal = int((value * 127 / 100) + 128)
        else:
            print("Invalid input. Value must be between -100 and 100.")
            return None
        
        return can_signal

    except ValueError:
        print("Invalid input. Please enter a valid integer.")
        return None

try:
    print("Enter a value from -100 to 100 (Ctrl+C to exit):")
    while True:
        user_input = input()
        can_signal = map_to_can(user_input)
        
        if can_signal is not None:
            # Create CAN message with the mapped CAN signal
            message = can.Message(arbitration_id=0x123, data=[can_signal], is_extended_id=False)

            # Send the CAN message
            bus.send(message)
            print("CAN Message sent:", message)

except KeyboardInterrupt:
    print("\nProgram terminated by user.")

finally:
    # Close the CAN bus
    bus.shutdown()
