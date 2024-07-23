#Brake_withCAN_slave_closedloop.py

import can

# Initialize CAN bus interface
bus = can.interface.Bus(channel='can0', bustype='socketcan')

try:
    while True:
        # Get user input for the value (0-1023)
        user_input = input("Enter a value from 0 to 1023 (or 'q' to quit): ").strip()
        
        # Check if user wants to quit
        if user_input.lower() == 'q':
            break
        
        # Convert input to integer
        user_input = int(user_input)
        
        # Check if input is within range
        if 0 <= user_input <= 1023:
            # Create CAN message with the user input value
            # Convert the input value to two bytes (16 bits)
            data = [(user_input >> 8) & 0xFF, user_input & 0xFF]
            message = can.Message(arbitration_id=0x123, data=data, is_extended_id=False)

            # Send the CAN message
            bus.send(message)
            print("CAN Message sent:", message)
        else:
            print("Invalid input. Value must be between 0 and 1023.")
            
except ValueError:
    print("Invalid input. Please enter a valid integer.")

except can.CanError:
    print("Error sending CAN message.")

finally:
    # Close the CAN bus
    bus.shutdown()

