import RPi.GPIO as GPIO
import can
import Adafruit_GPIO.SPI as SPI
import Adafruit_MCP3008
import threading
import signal
import sys
import time

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
extend_pwm = GPIO.PWM(EXTEND_PWM_PIN, PWM_FREQ)
retract_pwm = GPIO.PWM(RETRACT_PWM_PIN, PWM_FREQ)

# MCP3008 configuration
CLK = 21
MISO = 19
MOSI = 20
CS = 7
mcp = Adafruit_MCP3008.MCP3008(clk=CLK, cs=CS, miso=MISO, mosi=MOSI)

# Proportional gain (Kp)
Kp = 1

# Initialize CAN bus interface
bus = can.interface.Bus(channel='can0', bustype='socketcan')

brake_pos_set = 0

last_can_message = None

# Function to read position from MCP3008
def read_position():
    sensor_input = mcp.read_adc(0)
    brake_pos_real = int(sensor_input)
    return brake_pos_real

# Function to receive desired position from CAN bus
def can_receive():
    while True:
        print("4")
        #message = bus.recv()
        print("5")
        message = last_can_message if last_can_message is not None else bus.recv()
        can_input = (message.data[0] << 8) + message.data[1]
        brake_pos_set = int(can_input)
        print(brake_pos_set)
    return brake_pos_set

# Function to control PWM based on CAN input and position feedback
def brake(brake_pos_set, brake_pos_real):
    try:
        # Calculate error
        error = brake_pos_set - brake_pos_real

        # Calculate control value
        control_value = Kp * error

        # Map the control value from -100 to 100 for pwm application
        control_value = (control_value / 1023) * 100
        print("Control Value:", control_value)

        # Map control value to duty cycle
        if control_value > 0:  # Extension
            extend_pwm.ChangeDutyCycle(control_value)
            retract_pwm.ChangeDutyCycle(0)
        elif control_value < 0:  # Retraction
            extend_pwm.ChangeDutyCycle(0)
            retract_pwm.ChangeDutyCycle(-control_value)
        else:  # Stop
            extend_pwm.ChangeDutyCycle(0)
            retract_pwm.ChangeDutyCycle(0)

    except ValueError:
        print("Invalid input. Please enter a valid integer.")

try:
    # Start PWM
    extend_pwm.start(0)
    retract_pwm.start(0)

    print("Listening for CAN messages...")

    # Start a thread to continuously receive CAN messages
    can_thread = threading.Thread(target=can_receive)
    can_thread.daemon = True  # Daemonize the thread so it automatically exits when the main program ends
    can_thread.start()

    # Continuous loop to control the brake
    while True:
        # Loop to control the brake with the last received CAN message
        print("1")
        brake_pos_real = read_position()
        print("real = ",brake_pos_real)
        print("set = ",brake_pos_set)
        print("2")
        # Control brake based on desired and real positions
        brake(brake_pos_set, brake_pos_real)
        print("3")

        # Wait for 0.1 seconds before repeating
        time.sleep(0.1)
  

except KeyboardInterrupt:
    # Clean up GPIO
    extend_pwm.stop()
    retract_pwm.stop()
    GPIO.cleanup()

finally:
    # Close the CAN bus
    bus.shutdown()
