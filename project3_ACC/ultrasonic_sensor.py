# ultrasonic_sensor.py

import RPi.GPIO as GPIO
import time

# 초음파 센서 핀 설정
Tr = 11
Ec = 8

# GPIO 초기 설정
def init_ultrasonic():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(Tr, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(Ec, GPIO.IN)

# 거리 측정 함수
def checkdist():
    GPIO.output(Tr, GPIO.LOW)
    time.sleep(0.000002)
    GPIO.output(Tr, GPIO.HIGH)
    time.sleep(0.000015)
    GPIO.output(Tr, GPIO.LOW)

    while not GPIO.input(Ec):
        start = time.time()

    while GPIO.input(Ec):
        end = time.time()

    distance = ((end - start) * 340) / 2
    return distance

# GPIO 설정 초기화 함수
def cleanup():
    GPIO.cleanup()
