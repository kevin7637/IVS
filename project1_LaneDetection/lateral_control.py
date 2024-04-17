import time
import random
import numpy
from Adafruit_PCA9685 import PCA9685

MAX_PWM = 4095
HERTZ = 50
pwm = None

def calc_ticks(impulse_ms, hertz):
    cycle_ms = 1000.0 / hertz
    return int(MAX_PWM * impulse_ms/ cycle_ms + 0.5)


def translate(value, leftMin, leftMax, rightMin, rightMax):
    # Figure out how 'wide' each range is
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin

    # Convert the left range into a 0-1 range (float)
    valueScaled = float(value - leftMin) / float(leftSpan)

    # Convert the 0-1 range into a value in the right range.
    return rightMin + (valueScaled * rightSpan)


def lateral_set():
    global pwm
    duty_on = 0
    duty_off = 307
    servo_channel = 0

    print("PCA9685 servo control")

    pwm= PCA9685()
    pwm.set_pwm_freq(HERTZ)
    pwm.set_pwm(servo_channel, duty_on, duty_off)


def lateral_control(input_value):
    global pwm

    input_value = max(-10, min(input_value, 10))

    value = input_value * (-1)
    millis = translate(value, -10, 10, 1, 2)
    tick = calc_ticks(millis, HERTZ)

    servo_channel = 2
    duty_on = 0
    duty_off = tick

    pwm.set_pwm(servo_channel, duty_on, duty_off)
    # time.sleep(1)
