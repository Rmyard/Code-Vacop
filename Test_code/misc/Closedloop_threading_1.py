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

# Function to read position from MCP3008
def read_position():
    while not exit_signal.is_set():
        sensor_input = mcp.read_adc(0)
        brake_pos_real = int(sensor_input)
        print("Brake Position (Real):", brake_pos_real)
        time.sleep(1)

# Function to receive desired position from CAN bus
def can_receive():
    while not exit_signal.is_set():
        try:
            message = bus.recv()
            can_input = (message.data[0] << 8) + message.data[1]
            brake_pos_set = int(can_input)
            print("Brake Position (Set):", brake_pos_set)
            time.sleep(1)
        except can.CanError:
            pass  # Handle CAN errors gracefully

# Function to control PWM based on CAN input and position feedback
def brake():
    while not exit_signal.is_set():
        try:
            brake_pos_set = can_receive()
            brake_pos_real = read_position()

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
            time.sleep(1)

        except ValueError:
            print("Invalid input. Please enter a valid integer.")

# Signal handler for SIGINT (Ctrl+C)
def signal_handler(sig, frame):
    print("\nExiting program...")
    exit_signal.set()  # Set the exit signal to terminate threads
    sys.exit(0)

try:
    # Set up signal handler
    signal.signal(signal.SIGINT, signal_handler)

    # Start PWM
    extend_pwm.start(0)
    retract_pwm.start(0)

    print("Listening for CAN messages...")

    # Create exit signal to terminate threads
    exit_signal = threading.Event()

    # Create threads for reading position and receiving from CAN bus
    thread1 = threading.Thread(target=can_receive)
    thread2 = threading.Thread(target=read_position)
    thread3 = threading.Thread(target=brake)

    # Start threads
    thread1.start()
    thread2.start()
    thread3.start()

    # Join threads (optional, if you want the main thread to wait for these threads to finish)
    # thread1.join()
    # thread2.join()
    # thread3.join()

    # Keep the main thread running until interrupted
    while True:
        pass

except KeyboardInterrupt:
    # Clean up GPIO
    extend_pwm.stop()
    retract_pwm.stop()
    GPIO.cleanup()

finally:
    # Close the CAN bus
    bus.shutdown()
