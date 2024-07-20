#brake_control_slave.py
#12/03/24
#Rémi Myard
#This is a very basic code to control the brake actuator.
#arbitration_id

#000 OBU
#001 Brakes
#002 Steering
#003 Motors


import RPi.GPIO as GPIO
import can

# Initialize CAN bus interface
bus = can.interface.Bus(channel='can0', bustype='socketcan')

# Set GPIO mode to BCM
GPIO.setmode(GPIO.BCM)

# Set GPIO pins for PWM and direction
EXTEND_PIN = 17
RETRACT_PIN = 27
CURRENT_PIN = 22
POSITION_PIN = 23

GPIO.setup(EXTEND_PIN, GPIO.OUT)
GPIO.setup(RETRACT_PIN, GPIO.OUT)
GPIO.setup(CURRENT_PIN, GPIO.IN)
GPIO.setup(POSITION_PIN, GPIO.IN)

# Set PWM frequency (Hz)
PWM_FREQ = 100

# Create PWM instances
extend_pwm = GPIO.PWM(EXTEND_PIN, PWM_FREQ)
retract_pwm = GPIO.PWM(RETRACT_PIN, PWM_FREQ)

# Function to send a CAN message
def send_can_message(data):
    message = can.Message(arbitration_id=0x001, data=data) #id of 001 refer to the brake
    bus.send(message)
    print("CAN Message sent:", message)

# Function to receive a CAN message
def receive_can_message(data):
    # Receive CAN message
    message = bus.recv()
    force_setpoint = message.data[0] #force setpoint
    return (force_setpoint)


# Function to control PWM based on user input
def control_pwm(force_setpoint):
    try:
        # Convert input to integer
        value = int(force_setpoint)

        # Check if input is within range
        if -100 <= value <= 100:
            # Map value to duty cycle (0-100)
            duty_cycle = ((value + 100) / 200.0) * 100

            # Set PWM duty cycle for forward and backward directions
            if value >= 0:  # Forward
                extend_pwm.ChangeDutyCycle(duty_cycle)
                retract_pwm.ChangeDutyCycle(0)
            else:  # Backward
                extend_pwm.ChangeDutyCycle(0)
                retract_pwm.ChangeDutyCycle(duty_cycle)
        else:
            print("Invalid input. Value must be between -100 and 100.")
    except ValueError:
        print("Invalid input. Please enter a valid integer.")

try:
    # Start PWM
    extend_pwm.start(0)
    retract_pwm.start(0)

    # Continuous input
    while True:
        user_input = input("Enter a value from -100 to 100: ")
        control_pwm(user_input)

except KeyboardInterrupt:
    # Clean up GPIO
    extend_pwm.stop()
    retract_pwm.stop()
    GPIO.cleanup()
