import time
import RPi.GPIO as GPIO

Motor_A_EN = 4    # Motor_B_EN = 17
Motor_A_Pin1 = 14 # Motor_B_Pin1 = 27
Motor_A_Pin2 = 15 # Motor_B_Pin2 = 18

def motorStop(): # Motor stops
    GPIO.output(Motor_A_Pin1, GPIO.LOW)
    GPIO.output(Motor_A_Pin2, GPIO.LOW)
    GPIO.output(Motor_A_EN, GPIO.LOW)

def setup(): # Motor initialization
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(Motor_A_EN, GPIO.OUT)
    GPIO.setup(Motor_A_Pin1, GPIO.OUT)
    GPIO.setup(Motor_A_Pin2, GPIO.OUT)
    motorStop()

def move(speed, direction): # speed = 0~100
    if direction == 'forward':
        GPIO.output(Motor_A_Pin1, GPIO.HIGH)
        GPIO.output(Motor_A_Pin2, GPIO.LOW)
        GPIO.output(Motor_A_EN, GPIO.HIGH)
    elif direction == 'backward':
        GPIO.output(Motor_A_Pin1, GPIO.LOW)
        GPIO.output(Motor_A_Pin2, GPIO.HIGH)
        GPIO.output(Motor_A_EN, GPIO.HIGH)
    else:
        motorStop()

def destroy():
    motorStop()
    GPIO.cleanup() # Release resource

if __name__ == '__main__':
    try:
        speed_set = 100
        setup()
        move(speed_set, 'forward')
        int(input())
        time.sleep(1.3)
        motorStop()
        destroy()
    except KeyboardInterrupt:
        destroy()