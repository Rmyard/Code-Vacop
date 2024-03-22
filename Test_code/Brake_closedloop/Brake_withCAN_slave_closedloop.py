import RPi.GPIO as GPIO
import can
import os
import time
import Adafruit_GPIO.SPI as SPI
import Adafruit_MCP3008

# Set GPIO mode to BCM
GPIO.setmode(GPIO.BCM)

# Set GPIO pins for PWM
EXTEND_PWM_PIN = 17
RETRACT_PWM_PIN = 27
GPIO.setup(EXTEND_PWM_PIN, GPIO.OUT)
GPIO.setup(RETRACT_PWM_PIN, GPIO.OUT)

# Set PWM frequency (Hz)
PWM_FREQ = 100

# Create PWM instances
forward_pwm = GPIO.PWM(EXTEND_PWM_PIN, PWM_FREQ)
backward_pwm = GPIO.PWM(RETRACT_PWM_PIN, PWM_FREQ)

# MCP3008 configuration
CLK = 11
MISO = 9
MOSI = 10
CS = 8
mcp = Adafruit_MCP3008.MCP3008(clk=CLK, cs=CS, miso=MISO, mosi=MOSI)

# Proportional gain (Kp)
Kp = 1.0

# Function to read position from MCP3008
def read_position():
    return mcp.read_adc(0)

# Function to control PWM based on CAN input and position feedback
def control_position(can_input, desired_position):
    try:
        # Convert CAN input to integer
        value = int(can_input)

        # Calculate error (desired position - actual position)
        error = desired_position - read_position()

        # Calculate control value
        control_value = Kp * error

        # Map control value to duty cycle (0-100)
        if control_value >= 0:
            duty_cycle_forward = min(control_value, 100)
            duty_cycle_backward = 0
        else:
            duty_cycle_forward = 0
            duty_cycle_backward = min(-control_value, 100)

        # Set PWM duty cycle for forward and backward directions
        forward_pwm.ChangeDutyCycle(duty_cycle_forward)
        backward_pwm.ChangeDutyCycle(duty_cycle_backward)

    except ValueError:
        print("Invalid input. Please enter a valid integer.")

# Initialize CAN bus interface
os.system('sudo ip link set can0 up type can bitrate 1000000')
time.sleep(0.1)  # Wait for the interface to be initialized
bus = can.interface.Bus(channel='can0', bustype='socketcan')

try:
    # Start PWM
    forward_pwm.start(0)
    backward_pwm.start(0)

    print("Listening for CAN messages...")

    # Continuous listening for CAN messages
    while True:
        # Receive CAN message
        message = bus.recv()

        # Extract desired position from CAN message (assuming it's the first byte)
        desired_position = message.data[0]

        # Control position based on CAN input and position feedback
        control_position(desired_position, read_position())

except KeyboardInterrupt:
    # Clean up GPIO
    forward_pwm.stop()
    backward_pwm.stop()
    GPIO.cleanup()

finally:
    # Close the CAN bus
    bus.shutdown()
