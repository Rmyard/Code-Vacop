#Brake_withCAN_slave_openloop
#Pilot the motor controler with a pwm signal 
#(from 0 to 100 for retraction)
#(from -100 to 0 for extension)
#Message received from can0
#madeby:Rémi Myard

import RPi.GPIO as GPIO
import os



# Set GPIO mode to BCM
GPIO.setmode(GPIO.BCM)

# Set PWM frequency (Hz)
PWM_FREQ_BRAKE = 20000

# Set GPIO pins for PWM
EXTEND_PWM_PIN = 13
RETRACT_PWM_PIN = 12
EXTEND_EN_PIN = 23
RETRACT_EN_PIN = 24
GPIO.setup([EXTEND_PWM_PIN, RETRACT_PWM_PIN, EXTEND_EN_PIN, RETRACT_EN_PIN], GPIO.OUT)
extend_pwm = GPIO.PWM(EXTEND_PWM_PIN, PWM_FREQ_BRAKE)
retract_pwm = GPIO.PWM(RETRACT_PWM_PIN, PWM_FREQ_BRAKE)

#Enable pins :

GPIO.output(EXTEND_EN_PIN, GPIO.HIGH)
GPIO.output(RETRACT_EN_PIN, GPIO.HIGH)

def control_pwm(duty_cycle):
    try:
        # Map CAN input to duty cycle (0-100)
    
        
        if duty_cycle > 100:
            duty_cycle = 100  # Limit duty cycle to 100%

        if duty_cycle < -100:
            duty_cycle = -100
            
        print(duty_cycle)
        # Set PWM duty cycle for forward and backward directions
        if -100< value < 0:  # Extension
            extend_pwm.ChangeDutyCycle(-duty_cycle)
            retract_pwm.ChangeDutyCycle(0)
        elif 0 < value < 100:  # Retraction
            extend_pwm.ChangeDutyCycle(0)
            retract_pwm.ChangeDutyCycle(duty_cycle)
        else:  # Stop
            extend_pwm.ChangeDutyCycle(0)
            retract_pwm.ChangeDutyCycle(0)

    except ValueError:
        print("Invalid input. Please enter a valid integer.")

try:
    # Start PWM
    extend_pwm.start(0)
    retract_pwm.start(0)

    while True:
        value = int(input("input de -100 a 100 (negatif = sortie),(positif = rentrée) :"))
        control_pwm(value)

except KeyboardInterrupt:
    # Clean up GPIO
    extend_pwm.stop()
    retract_pwm.stop()
    GPIO.cleanup()
