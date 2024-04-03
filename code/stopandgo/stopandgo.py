import time
import RPi.GPIO as GPIO
import board

Motor_A_EN = 4    # Motor_B_EN = 17
Motor_A_Pin1 = 14 # Motor_B_Pin1 = 27
Motor_A_Pin2 = 15 # Motor_B_Pin2 = 18
TRIG = 11
ECHO = 8



def setup(): # Motor initialization
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(Motor_A_EN, GPIO.OUT)
    GPIO.setup(Motor_A_Pin1, GPIO.OUT)
    GPIO.setup(Motor_A_Pin2, GPIO.OUT)
    GPIO.setup(TRIG, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(ECHO, GPIO.IN)
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
        
def motorStop(): # Motor stops
    GPIO.output(Motor_A_Pin1, GPIO.LOW)
    GPIO.output(Motor_A_Pin2, GPIO.LOW)
    GPIO.output(Motor_A_EN, GPIO.LOW)
    
def destroy():
    motorStop()
    GPIO.cleanup() # Release resource

def checkdist():
    # 초음파 센서로 거리 측정
    for i in range(5):
        GPIO.output(TRIG, GPIO.LOW)
        time.sleep(0.000002)
        GPIO.output(TRIG, GPIO.HIGH)
        time.sleep(0.000015)
        GPIO.output(TRIG, GPIO.LOW)
        
        while not GPIO.input(ECHO):
            pass
        t1 = time.time()
        
        while GPIO.input(ECHO):
            pass
        t2 = time.time()
        
        dist = (t2 - t1) * 340 / 2
        
        if dist > 9 and i < 4:
            continue
        else:
            return dist
    
if __name__ == '__main__':
    setup()
    try:
        while True:
            distance = checkdist() * 100
            print("Distance: %.2f cm" % distance)
            
            if distance < 20:  # 일정 거리 내에 장애물이 있을 경우
                motorStop()
            else:
                move(100,'forward')
                           
            time.sleep(0.1)
    
    except KeyboardInterrupt:
        GPIO.cleanup()