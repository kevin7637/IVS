import RPi.GPIO as GPIO
from Adafruit_PCA9685 import PCA9685
import time

Motor_B_EN = 4    
Motor_B_Pin1 = 14 
Motor_B_Pin2 = 15 
Motor_A_EN = 17
Motor_A_Pin1 = 27
Motor_A_Pin2 = 18

Tr = 11           
Ec = 8  
HERTZ = 50

def setup(): # Motor initialization
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(Motor_A_EN, GPIO.OUT)
    GPIO.setup(Motor_A_Pin1, GPIO.OUT)
    GPIO.setup(Motor_A_Pin2, GPIO.OUT)
    motorStop()
    
    pwm = PCA9685()
    pwm.set_pwm_freq(HERTZ)
    
    
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(Tr, GPIO.OUT)
    GPIO.setup(Ec, GPIO.IN)
    
def motorStop(): # Motor stops
    GPIO.output(Motor_A_Pin1, GPIO.LOW)
    GPIO.output(Motor_A_Pin2, GPIO.LOW)
    GPIO.output(Motor_A_EN, GPIO.LOW)
    
def destroy():
    motorStop()
    GPIO.cleanup()
    
def move(speed, direction): # speed = 0~100
    if direction == 'backward':
        GPIO.output(Motor_A_Pin1, GPIO.HIGH)
        GPIO.output(Motor_A_Pin2, GPIO.LOW)
        GPIO.output(Motor_A_EN, GPIO.HIGH)
    elif direction == 'forward':
        GPIO.output(Motor_A_Pin1, GPIO.LOW)
        GPIO.output(Motor_A_Pin2, GPIO.HIGH)
        GPIO.output(Motor_A_EN, GPIO.HIGH)
    else:
        motorStop()

def checkdist():
    # 초음파 센서로 거리 측정
    for i in range(5):
        GPIO.output(Tr, GPIO.LOW)
        time.sleep(0.000002)
        GPIO.output(Tr, GPIO.HIGH)
        time.sleep(0.000015)
        GPIO.output(Tr, GPIO.LOW)
        
        while not GPIO.input(Ec):
            pass
        t1 = time.time()
        
        while GPIO.input(Ec):
            pass
        t2 = time.time()
        
        dist = (t2 - t1) * 340 / 2
        
        if dist > 9 and i < 4:
            continue
        else:
            return dist
