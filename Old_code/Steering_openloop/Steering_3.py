#use libary time to generate steps
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


def read_steer_position():
    steer_pos_real = int(mcp.read_adc(0)) #third pin of the mcp on the test bench
    print("steer_pos_real = ",steer_pos_real)
    return steer_pos_real

def steer(steer_pos_set):
    steer_pos_real = read_steer_position()
    print("steer_pos_real = ",steer_pos_real)
    print("steer_pos_set = ",steer_pos_set)
    error = steer_pos_set - steer_pos_real
        
    #safety mechanism
    if steer_pos_real < 10:
        GPIO.output(EN_PIN, GPIO.HIGH) #Desactivate the motor in case of reaching min value
        print("steer min value reached, motor blocked")
    elif steer_pos_real > 1013: #Desactivate the motor in case of reaching max value
        GPIO.output(EN_PIN, GPIO.HIGH)
        print("steer max value reached, motor blocked")
    else:
        GPIO.output(EN_PIN, GPIO.LOW)

    #Define direction of spin
    if error > 10:
        GPIO.output(DIR_PIN, GPIO.HIGH)
        #send pulse
        GPIO.output(PUL_PIN, GPIO.HIGH)
        time.sleep(0.0001)
        GPIO.output(PUL_PIN, GPIO.LOW)
        time.sleep(0.0001)

    elif error < -10:
        GPIO.output(DIR_PIN, GPIO.LOW)
        #send pulse
        GPIO.output(PUL_PIN, GPIO.HIGH)
        time.sleep(0.0001)
        GPIO.output(PUL_PIN, GPIO.LOW)
        time.sleep(0.0001)

    else:
        GPIO.output(EN_PIN, GPIO.HIGH)
    
    #Check real value
    steer_pos_real = read_steer_position()
    error = steer_pos_set - steer_pos_real

try:
    while True:
        steer(400)

finally:
    GPIO.cleanup()

    
