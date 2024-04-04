import cv2
import numpy as np
import RPi.GPIO as GPIO

PWMA = 17
AIN1   =  27
AIN2   =  18

PWMB = 4
BIN1   = 14
BIN2  =  15
    
def motor_go(speed):
    L_Motor.ChangeDutyCycle(speed)
    GPIO.output(AIN2,True)#AIN2
    GPIO.output(AIN1,False) #AIN1
    R_Motor.ChangeDutyCycle(speed)
    GPIO.output(BIN2,True)#BIN2
    GPIO.output(BIN1,False) #BIN1
    
def motor_right(speed):
    L_Motor.ChangeDutyCycle(speed)
    GPIO.output(AIN2,True)#AIN2
    GPIO.output(AIN1,False) #AIN1
    R_Motor.ChangeDutyCycle(0)
    GPIO.output(BIN2,False)#BIN2
    GPIO.output(BIN1,True) #BIN1
    
def motor_left(speed):
    L_Motor.ChangeDutyCycle(0)
    GPIO.output(AIN2,False)#AIN2
    GPIO.output(AIN1,True) #AIN1
    R_Motor.ChangeDutyCycle(speed)
    GPIO.output(BIN2,True)#BIN2
    GPIO.output(BIN1,False) #BIN1
    
def motor_stop():
    GPIO.output(AIN1,False)
    GPIO.output(AIN2,True)
    L_Motor.ChangeDutyCycle(0)
    GPIO.output(BIN1,False)
    GPIO.output(BIN2,True)
    R_Motor.ChangeDutyCycle(0)
        
GPIO.setwarnings(False) 
GPIO.setmode(GPIO.BCM)
GPIO.setup(AIN2,GPIO.OUT)
GPIO.setup(AIN1,GPIO.OUT)
GPIO.setup(PWMA,GPIO.OUT)

GPIO.setup(BIN1,GPIO.OUT)
GPIO.setup(BIN2,GPIO.OUT)
GPIO.setup(PWMB,GPIO.OUT)

SW1 = 5
SW2 = 6
SW3 = 13
SW4 = 19

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(SW1,GPIO.IN,pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(SW2,GPIO.IN,pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(SW3,GPIO.IN,pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(SW4,GPIO.IN,pull_up_down=GPIO.PUD_DOWN)

LED1 = 26
LED2 = 16
LED3 = 20
LED4 = 21

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(LED1,GPIO.OUT)
GPIO.setup(LED2,GPIO.OUT)
GPIO.setup(LED3,GPIO.OUT)
GPIO.setup(LED4,GPIO.OUT)

L_Motor= GPIO.PWM(PWMA,100)
L_Motor.start(0)

R_Motor = GPIO.PWM(PWMB,100)
R_Motor.start(0)

def main():
    camera = cv2.VideoCapture(0)
    camera.set(3,160) 
    camera.set(4,120)

    while( camera.isOpened() ):
        ret, frame = camera.read()
        frame = cv2.flip(frame,-1)
        cv2.imshow('normal',frame)
        
        crop_img =frame[30:100, 0:160]
        
        gray = cv2.cvtColor(crop_img, cv2.COLOR_BGR2GRAY)
    
        blur = cv2.GaussianBlur(gray,(5,5),0)
        
        ret,thresh1 = cv2.threshold(blur,60,255,cv2.THRESH_BINARY_INV)
        
        mask = cv2.erode(thresh1, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        cv2.imshow('mask',mask)
    
        contours,hierarchy = cv2.findContours(mask.copy(), 1, cv2.CHAIN_APPROX_NONE)
        
        if len(contours) > 0:
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c)
            
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            
            if cx <= 65:              
                print("Turn Left!")
                motor_left(40)
                GPIO.output(LED1,GPIO.HIGH)
                GPIO.output(LED2,GPIO.LOW)
                GPIO.output(LED3,GPIO.HIGH)
                GPIO.output(LED4,GPIO.LOW)
            elif cx >= 90:
                print("Turn Right")
                motor_right(40)
                GPIO.output(LED1,GPIO.LOW)
                GPIO.output(LED2,GPIO.HIGH)
                GPIO.output(LED3,GPIO.LOW)
                GPIO.output(LED4,GPIO.HIGH)
            else:
                print("go")
                motor_go(40)
                GPIO.output(LED1,GPIO.HIGH)
                GPIO.output(LED2,GPIO.HIGH)
                GPIO.output(LED3,GPIO.LOW)
                GPIO.output(LED4,GPIO.LOW)
        
            if GPIO.input(SW1) == 1 or GPIO.input(SW2) == 1 or GPIO.input(SW3) == 1 or GPIO.input(SW4) == 1 :
                motor_stop()
                GPIO.output(LED1,GPIO.LOW)
                GPIO.output(LED2,GPIO.LOW)
                GPIO.output(LED3,GPIO.LOW)
                GPIO.output(LED4,GPIO.LOW)
                break

        if cv2.waitKey(1) == ord('q'):
            break
    
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
    GPIO.cleanup()