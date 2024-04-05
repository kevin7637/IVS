from control.PID import *
from trejectory.trejectory import *
from move.move import*
from Adafruit_PCA9685 import PCA9685

HERTZ = 50
FAIL = -1
WIDTH = 0
pwm = PCA9685()
pwm.set_pwm_freq(HERTZ)
setup()
measure = 0
step_time = 0.001
controller = PD_Controller(measure, step_time)
speed_set = 50
tick = 300
opt = 0
cnt = 0
pwm.set_pwm(0, 0, tick)

def distance_stop(distance):
    if distance < 0.5:
        motorStop()
        time.sleep(1) 
    else:
        speed_set = 50
        move(speed_set, 'forward')
        
if __name__ == "__main__":
    while True:
        try:
            distance = detectObstacle()
            distance_stop(distance)
            #좌표 = camera
            coord = camera()
            controller.ControllerInput(coord[])
            yaw_controll(controller.u)
        except KeyboardInterrupt:
            destroy()
            

                
