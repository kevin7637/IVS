import time
import RPi.GPIO as GPIO
from Adafruit_PCA9685 import PCA9685
import cv2

HERTZ = 50
FAIL = -1
BASIC_SPEED = 20
MAX_SPEED = 100
CNT = 0
pwm = PCA9685()
pwm.set_pwm_freq(HERTZ)
camera = cv2.VideoCapture(0) 
camera.set(3,640)  
camera.set(4,480)  
Motor_B_EN = 4    
Motor_B_Pin1 = 14 
Motor_B_Pin2 = 15 
Motor_A_EN = 17
Motor_A_Pin1 = 27
Motor_A_Pin2 = 18
line_pin_right = 19
line_pin_middle = 16
line_pin_left = 20
TRIG_PIN = 11           
ECHO_PIN = 8  

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(Motor_A_EN, GPIO.OUT)
GPIO.setup(Motor_A_Pin1, GPIO.OUT)
GPIO.setup(Motor_A_Pin2, GPIO.OUT)
pwm_A = GPIO.PWM(Motor_A_EN, HERTZ)
pwm_A.start(0)
pwm = PCA9685()
pwm.set_pwm_freq(HERTZ)

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG_PIN, GPIO.OUT)
GPIO.setup(ECHO_PIN, GPIO.IN)

GPIO.setup(line_pin_right, GPIO.IN)
GPIO.setup(line_pin_middle, GPIO.IN)
GPIO.setup(line_pin_left, GPIO.IN)

def detectObstacle():
    # 초음파 센서로 거리 측정
    for i in range(5):
        GPIO.output(TRIG_PIN, GPIO.LOW)
        time.sleep(0.1)
        GPIO.output(TRIG_PIN, GPIO.HIGH)
        time.sleep(0.00001)
        GPIO.output(TRIG_PIN, GPIO.LOW)
        
        while not GPIO.input(ECHO_PIN):
            pass
        t1 = time.time()
        
        while GPIO.input(ECHO_PIN):
            pass
        t2 = time.time()
        
        dist = (t2 - t1) * 340 / 2
        
        if dist > 10 and i < 4:
            continue
        else:
            return dist

def distance_stop(distance):
    if distance < 0.2:
        motorStop()
        time.sleep(0.1) 
    else:
        speed_set = 20
        move(speed_set, 'forward')
def move(speed, direction): # speed = 0~100
    if direction == 'backward':
        GPIO.output(Motor_A_Pin1, GPIO.HIGH)
        GPIO.output(Motor_A_Pin2, GPIO.LOW)
        #GPIO.output(Motor_A_EN, GPIO.HIGH)
    elif direction == 'forward':
        GPIO.output(Motor_A_Pin1, GPIO.LOW)
        GPIO.output(Motor_A_Pin2, GPIO.HIGH)
        #GPIO.output(Motor_A_EN, GPIO.HIGH)
    else:
        motorStop()
        return
    if speed > 100:
        speed = 100
    elif speed < 0:
        speed = 0
    pwm_A.ChangeDutyCycle(speed)
    
def motorStop(): # Motor stops
    GPIO.output(Motor_A_Pin1, GPIO.LOW)
    GPIO.output(Motor_A_Pin2, GPIO.LOW)
    GPIO.output(Motor_A_EN, GPIO.LOW) 
    
def destroy():
    motorStop()
    GPIO.cleanup()    

    
if __name__ == "__main__":
    servo_tick = 300
    pwm.set_pwm(0, 0, servo_tick)
    motorStop()
    while True:
        try:
            distance = detectObstacle()
            print("sonic",distance)
            if distance < 0.30:
                print("STOP")
                motorStop()
                time.sleep(1) 
            else:
                speed_set = BASIC_SPEED
                move(speed_set, 'forward')
                time.sleep(0.1)
                if CNT == 0:
                    speed_set = MAX_SPEED
                    move(speed_set, 'forward')
                    time.sleep(0.1)
                    servo_tick = 390
                    pwm.set_pwm(0, 0, servo_tick)
                    time.sleep(1)
                    CNT += 1
                else:
                    servo_tick = 300
                    pwm.set_pwm(0, 0, servo_tick)
                
            
        except KeyboardInterrupt:
            cv2.destroyAllWindows() 
            destroy()