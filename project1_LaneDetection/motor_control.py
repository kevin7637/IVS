import time
import RPi.GPIO as GPIO

Motor_A_EN = 4  # Motor_B_EN = 17
Motor_A_Pin1 = 14 # Motor_B_Pin1 = 27
Motor_A_Pin2 = 15 # Motor_B_Pin2 = 18

# PWM 핀 설정
PWM_FREQ = 1000  # PWM 주파수 (Hz)
PWM_SPEED_PIN = 18  # 모터 속도 제어를 위한 PWM 핀

pwm = None  # 전역 PWM 객체 변수

def motorStop():  # Motor stops
    GPIO.output(Motor_A_Pin1, GPIO.LOW)
    GPIO.output(Motor_A_Pin2, GPIO.LOW)
    GPIO.output(Motor_A_EN, GPIO.LOW)

def setup():  # Motor initialization
    global pwm  # 전역 변수로 선언
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(Motor_A_EN, GPIO.OUT)
    GPIO.setup(Motor_A_Pin1, GPIO.OUT)
    GPIO.setup(Motor_A_Pin2, GPIO.OUT)
    motorStop()

    # PWM 설정
    pwm = GPIO.PWM(Motor_A_EN, PWM_FREQ)
    pwm.start(0)  # PWM 시작 (초기 듀티 사이클: 0)

def move(speed, direction):
    global pwm  # 전역 변수 사용
    # Speed = 0~100
    speed = max(0, min(speed, 100))

    if direction == 'forward':
        GPIO.output(Motor_A_Pin1, GPIO.HIGH)
        GPIO.output(Motor_A_Pin2, GPIO.LOW)
        pwm.ChangeDutyCycle(speed)  # PWM 듀티 사이클 변경

    elif direction == 'backward':
        GPIO.output(Motor_A_Pin1, GPIO.LOW)
        GPIO.output(Motor_A_Pin2, GPIO.HIGH)
        pwm.ChangeDutyCycle(speed)  # PWM 듀티 사이클 변경

    else:
        motorStop()

def destroy():
    motorStop()
    GPIO.cleanup()
