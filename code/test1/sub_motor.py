import time
import random
from Adafruit_PCA9685 import PCA9685
PIN_BASE = 300
MAX_PWM = 4095
HERTZ = 50
def calc_ticks(impulse_ms, hertz):
    cycle_ms = 1000.0 / hertz
    return int(MAX_PWM * impulse_ms / cycle_ms + 0.5)
def map_value(input_value, min_value, max_value):
    return input_value * max_value + (1 - input_value) * min_value

def main():
    print("PCA9685 servo example")
    print("Connect a servo to any pin. It will rotate to random angles")
    pwm = PCA9685()
    pwm.set_pwm_freq(HERTZ)
    tick = 300
    pwm.set_pwm(0, 0, tick)
    time.sleep(2)

if __name__ == "__main__":
    main()