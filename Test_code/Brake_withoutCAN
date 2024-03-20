#Brake_without_can
#Code to send a pwm signal to the motor controller (BTS7960) with a user input. 
#Rémi Myard
import RPi.GPIO as GPIO

# Set GPIO mode to BCM
GPIO.setmode(GPIO.BCM)

# Set GPIO pins for PWM and direction
FORWARD_PWM_PIN = 17
BACKWARD_PWM_PIN = 27
GPIO.setup(FORWARD_PWM_PIN, GPIO.OUT)
GPIO.setup(BACKWARD_PWM_PIN, GPIO.OUT)

# Set PWM frequency (Hz)
PWM_FREQ = 100

# Create PWM instances
forward_pwm = GPIO.PWM(FORWARD_PWM_PIN, PWM_FREQ)
backward_pwm = GPIO.PWM(BACKWARD_PWM_PIN, PWM_FREQ)

# Function to control PWM based on user input
def control_pwm(user_input):
    try:
        # Convert input to integer
        value = int(user_input)

        # Check if input is within range
        if -100 <= value <= 100:
            # Map value to duty cycle (0-100)
            duty_cycle = abs(value)

            # Set PWM duty cycle for forward and backward directions
            if value >= 0:  # Forward
                forward_pwm.ChangeDutyCycle(duty_cycle)
                backward_pwm.ChangeDutyCycle(0)
            else:  # Backward
                forward_pwm.ChangeDutyCycle(0)
                backward_pwm.ChangeDutyCycle(duty_cycle)
        else:
            print("Invalid input. Value must be between -100 and 100.")
    except ValueError:
        print("Invalid input. Please enter a valid integer.")

try:
    # Start PWM
    forward_pwm.start(0)
    backward_pwm.start(0)

    # Continuous input
    while True:
        user_input = input("Enter a value from -100 to 100: ")
        control_pwm(user_input)

except KeyboardInterrupt:
    # Clean up GPIO
    forward_pwm.stop()
    backward_pwm.stop()
    GPIO.cleanup()
