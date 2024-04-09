import time
import RPi.GPIO as GPIO
from Adafruit_PCA9685 import PCA9685
import cv2

HERTZ = 50
FAIL = -1
BASIC_SPEED = 28
CNT = 0
pwm = PCA9685()
pwm.set_pwm_freq(HERTZ)
camera = cv2.VideoCapture(0) 
camera.set(3,640)  
camera.set(4,480)  
servo_tick = 300
pwm.set_pwm(0, 0, servo_tick)
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
GPIO.setup(Tr, GPIO.OUT)
GPIO.setup(Ec, GPIO.IN)

GPIO.setup(line_pin_right, GPIO.IN)
GPIO.setup(line_pin_middle, GPIO.IN)
GPIO.setup(line_pin_left, GPIO.IN)

def detectObstacle(num_measurements=3, timeout=0.1):
    distances = []
    
    for _ in range(num_measurements):
        GPIO.output(TRIG_PIN, GPIO.LOW)
        time.sleep(0.000002)    
        GPIO.output(TRIG_PIN, GPIO.HIGH)
        time.sleep(0.00001)
        GPIO.output(TRIG_PIN, GPIO.LOW)
        
        pulse_start = time.time()
        while not GPIO.input(ECHO_PIN):
            if time.time() - pulse_start > timeout:
                break
        
        pulse_end = time.time()
        while GPIO.input(ECHO_PIN):
            if time.time() - pulse_end > timeout:
                break
        
        pulse_duration = pulse_end - pulse_start
        distance = pulse_duration * 17150  # 340 m/s / 2
        distances.append(distance)
    
    avg_distance = sum(distances) / len(distances)
    return avg_distance

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
    pwm_A.ChangeDutyCycle(0)    
    
def destroy():
    motorStop()
    GPIO.cleanup()    

    
if __name__ == "__main__":
    while True:
        try:
            #move(speed_set, 'forward')
            distance = detectObstacle()
            ti = time.time()
            if distance < 0.15:
                motorStop()
            else: 
                speed_set = BASIC_SPEED
                move(speed_set, 'forward')
            
        except KeyboardInterrupt:
            cv2.destroyAllWindows() 
            destroy()