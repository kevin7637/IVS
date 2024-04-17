import time
import RPi.GPIO as GPIO

Motor_A_EN    = 17

Motor_A_Pin1  = 27
Motor_A_Pin2  = 18

Dir_forward   = 0
Dir_backward  = 1

left_forward  = 0
left_backward = 1

right_forward = 0
right_backward= 1

pwn_A = 0



def Stop():#Motor stops
	GPIO.output(Motor_A_Pin1, GPIO.LOW)
	GPIO.output(Motor_A_Pin2, GPIO.LOW)
	GPIO.output(Motor_A_EN, GPIO.LOW)


def setup():#Motor initialization
	global pwm_A
	GPIO.setwarnings(False)
	GPIO.setmode(GPIO.BCM)
	GPIO.setup(Motor_A_EN, GPIO.OUT)
	GPIO.setup(Motor_A_Pin1, GPIO.OUT)
	GPIO.setup(Motor_A_Pin2, GPIO.OUT)

	Stop()
	try:
		pwm_A = GPIO.PWM(Motor_A_EN, 1000)
	except:
		pass

def motor_right(status, direction, speed):#Motor 1 positive and negative rotation
	if status == 0: # stop
		GPIO.output(Motor_A_Pin1, GPIO.LOW)
		GPIO.output(Motor_A_Pin2, GPIO.LOW)
		GPIO.output(Motor_A_EN, GPIO.LOW)
	else:
		if direction == Dir_forward:#
			GPIO.output(Motor_A_Pin1, GPIO.HIGH)
			GPIO.output(Motor_A_Pin2, GPIO.LOW)
			pwm_A.start(100)
			pwm_A.ChangeDutyCycle(speed)
		elif direction == Dir_backward:
			GPIO.output(Motor_A_Pin1, GPIO.LOW)
			GPIO.output(Motor_A_Pin2, GPIO.HIGH)
			pwm_A.start(100)
			pwm_A.ChangeDutyCycle(speed)
	return direction


def move(speed, direction, radius=0.6):   # 0 < radius <= 1  
	#speed = 100
	if direction == 'forward':
		motor_right(1, right_forward, speed)
	elif direction == 'backward':
		motor_right(1, right_backward, speed)
	elif direction == 'no':
		motorStop()
	else:
		pass

def Forward(speed_set):
    move(speed_set, 'forward', 0.8)

def Backward(speed):
    move(speed_set, 'backward', 0.8)


def destroy():
	motorStop()
	GPIO.cleanup()             # Release resource


if __name__ == '__main__':
	try:
		speed_set = 60
		setup()
		move(speed_set, 'forward', 0.8)
		time.sleep(1.3)
		move(speed_set, 'backward', 0.8)
		time.sleep(1.3)
		motorStop()
		destroy()
	except KeyboardInterrupt:
		destroy()
