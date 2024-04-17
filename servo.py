import time
import Adafruit_PCA9685

# Initialize the PCA9685 board
pwm = Adafruit_PCA9685.PCA9685()

# Set PWM frequency (50Hz is typical for servos)
pwm.set_pwm_freq(50)

# Define the servo channel
servo_channel = 4
servo_channel_2 = 1

def set_servo_position(degrees):
    pulse = int(204.8 + degrees * 2.048) # Convert degrees to PWM pulse length
    pwm.set_pwm(servo_channel, 0, pulse)

def HardTurnright():
    pwm.set_pwm(servo_channel, 0, 240)  # Set servo position for hard turn right
    #pwm.set_pwm(servo_channel_2, 0, 143)  # Set servo position for hard turn right

def Turnright():
    pwm.set_pwm(servo_channel, 0, 270)  # Set servo position for turn right
    #pwm.set_pwm(servo_channel_2, 0, 179)  # Set servo position for hard turn right

def Go():
    pwm.set_pwm(servo_channel, 0, 307)  # Set servo position for going straight
    #pwm.set_pwm(servo_channel_2, 0, 307)  # Set servo position for hard turn right

def Turnleft():
    pwm.set_pwm(servo_channel, 0, 320)  # Set servo position for turn left
    #pwm.set_pwm(servo_channel_2, 0, 410)  # Set servo position for hard turn right

def HardTurnleft():
    pwm.set_pwm(servo_channel, 0, 360)  # Set servo position for hard turn left
    #pwm.set_pwm(servo_channel_2, 0, 512)  # Set servo position for hard turn right

if __name__ == '__main__':
    try:
        while True:
            pwm.set_pwm(servo_channel, 0, 400)  # Set servo position for going straight
            time.sleep(1)
            #pwm.set_pwm(servo_channel, 0, 410)
            #time.sleep(1)
            #pwm.set_pwm(servo_channel, 0, 307)
            #time.sleep(1)
            #pwm.set_pwm(servo_channel, 0, 200)
            #time.sleep(1)
            #pwm.set_pwm(servo_channel, 0, 100)
            #time.sleep(1)
            #pwm.set_pwm(servo_channel, 0, 307)
            #time.sleep(1)

    except KeyboardInterrupt:
        pwm.set_pwm(servo_channel, 0, 307)  # Set servo position for going straight
        pwm.set_pwm(servo_channel_2, 0, 307)
        pwm.set_pwm(0, 0, 100)
        pwm.deinit()
