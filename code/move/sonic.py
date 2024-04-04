import time
import RPi.GPIO as GPIO
import math

Servo_PIN = 2     # 초음파 센서 서보 모터 제어 핀
Tr = 11           # 초음파 센서 Trig 핀
Ec = 8            # 초음파 센서 Echo 핀

def detectObstacle():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(Tr, GPIO.OUT)
    GPIO.setup(Ec, GPIO.IN)

    tick = 350 # 초음파 센서가 정면을 향하도록 고정

    while True:
        GPIO.output(Tr, False)
        time.sleep(0.000002)
        GPIO.output(Tr, True)
        time.sleep(0.00001)
        GPIO.output(Tr, False)
        
        while GPIO.input(Ec):
            pass
        pulse_start = time.time()
        while GPIO.input(Ec):
            pass
        pulse_end = time.time()
        
        pulse_duration = pulse_end - pulse_start
        distance = pulse_duration * 17150 #cm/s
        distance = round(distance, 2)
        
        if distance < 50: # 100cm 이내에 장애물이 있을 경우
            return distance
        else:
            return None