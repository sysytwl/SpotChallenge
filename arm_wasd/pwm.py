import RPi.GPIO as GPIO
import time

# Set up the GPIO mode
GPIO.setmode(GPIO.BCM)

# Set the PWM pin
pwm_pin = 12
GPIO.setup(pwm_pin, GPIO.OUT)

# Set up PWM frequency and start it
pwm = GPIO.PWM(pwm_pin, 500)  # 500 Hz frequency
pwm.start(20)  # Start

def connector_open():
    for duty_cycle in range(20, 60, 1):
        pwm.ChangeDutyCycle(duty_cycle)
        time.sleep(0.01)

def connector_close():
    for duty_cycle in range(60, 20, -1):
        pwm.ChangeDutyCycle(duty_cycle)
        time.sleep(0.01)

def clean_up():
    # Clean up
    pwm.stop()
    GPIO.cleanup()
