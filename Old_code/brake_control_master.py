#brake_control_master.py
#12/03/24
#Rémi Myard
#This is a very basic code to control the brake actuator.
#arbitration_id

#000 OBU
#001 Brakes
#002 Steering
#003 Motors
import can
import os

# Initialize CAN bus interface
os.system("sudo ip link set can0 up type can bitrate 1000000")
bus = can.interface.Bus(channel='can0', bustype='socketcan')

try:
    # Get user input for the value (0-255)
    user_input = int(input("Enter a value from 0 to 255: "))
    
    # Check if input is within range
    if 0 <= user_input <= 255:
        # Create CAN message with the user input value
        message = can.Message(arbitration_id=0x123, data=[user_input], is_extended_id=False)

        # Send the CAN message
        bus.send(message)
        print("CAN Message sent:", message)
    else:
        print("Invalid input. Value must be between 0 and 255.")
        
except ValueError:
    print("Invalid input. Please enter a valid integer.")

except can.CanError:
    print("Error sending CAN message.")

finally:
    # Close the CAN bus
    bus.shutdown()
