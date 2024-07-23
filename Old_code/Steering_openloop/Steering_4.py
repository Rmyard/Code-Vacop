#use pwm to generate pulses for steps
import RPi.GPIO as GPIO
import time
import Adafruit_GPIO.SPI as SPI
import Adafruit_MCP3008

################################
# RPi and Motor Pre-allocations
################################

# MCP3008 configuration
CLK = 21
MISO = 19
MOSI = 20
CS = 7
mcp = Adafruit_MCP3008.MCP3008(clk=CLK, cs=CS, miso=MISO, mosi=MOSI)

# Define GPIO pins
GPIO.setmode(GPIO.BCM)
DIR_PIN= 17 # Direction (DIR) GPIO Pin
PUL_PIN = 26 # Step GPIO Pin
EN_PIN = 16 # Enable GPIO Pin
GPIO.setup([DIR_PIN, PUL_PIN, EN_PIN], GPIO.OUT)

# Set PWM frequency (Hz)
PWM_FREQ = 1000
pulse = GPIO.PWM(PUL_PIN, PWM_FREQ)
KP_STEER = 1
STEER_LEFT_LIMIT = 0
STEER_RIGHT_LIMIT = 1023

def read_steer_position():
    steer_pos_real = int(mcp.read_adc(0)) #third pin of the mcp on the test bench
    print("steer_pos_real = ",steer_pos_real)
    return steer_pos_real

def steer(steer_pos_set):
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
    else:
        GPIO.output(STEER_EN_PIN, GPIO.LOW)

    #Define direction of spin and send PWM pulses
    if control_value > STEER_THRESHOLD:
        
        GPIO.output(STEER_DIR_PIN, GPIO.HIGH)
        #send pulse
        pulse.ChangeDutyCycle(50)

    elif control_value < -STEER_THRESHOLD:
       
        GPIO.output(STEER_DIR_PIN, GPIO.LOW)
        #send pulse
        pulse.ChangeDutyCycle(50)

    else:
        pulse.ChangeDutyCycle(0)

# Function to deactivate the braking motor
def steer_enable(argument):
    if argument == False: #if the argument is false, the brake is deactivated
        GPIO.output(EN_PIN, GPIO.LOW)
        print("brake deactivated")

    else:
        GPIO.output(EN_PIN, GPIO.HIGH)
        print("brake activated")

try:
    # Start PWM
    
    while True:
        steer_enable(True)
        steer(1000)
        time.sleep


finally:
    GPIO.cleanup()

    
