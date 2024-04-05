from control.PID import *
from trejectory.trejectory import *
from move.move import*
from Adafruit_PCA9685 import PCA9685
from camera.camera import *
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

global last_centroid
global camera
camera = cv2.VideoCapture(0) 
camera.set(3,640)  
camera.set(4,480)  


#camera.release()



def distance_stop(distance):
    if distance < 0.5:
        motorStop()
        time.sleep(0.1) 
    else:
        speed_set = 50
        move(speed_set, 'forward')


  
if __name__ == "__main__":
    while True:
        try:
            #distance = detectObstacle()
            #distance_stop(distance)
            #좌표 = camera
            _, image = camera.read()
            centroid,image = point_tracking(image)
            cv2.imshow("image",image)
            if centroid:
                last_centroid = centroid
            print(last_centroid)
            controller.ControllerInput(-200)
            servo_tick = yaw_controll(controller.u,320)
            print(servo_tick)
            pwm.set_pwm(0, 0, servo_tick)
        except KeyboardInterrupt:
            cv2.destroyAllWindows() 
            camera.release()
            destroy()
            

'''
from control.PID import *
from trejectory.trejectory import *
from move.move import*
from Adafruit_PCA9685 import PCA9685
from camera.camera import *
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

global last_centroid
global camera
camera = cv2.VideoCapture(0) 
camera.set(3,640)  
camera.set(4,480)  


#camera.release()



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
            print(distance)
            distance_stop(distance)
            #좌표 = camera
            _, image = camera.read()
            centroid = point_tracking(image)
            if centroid:
                last_centroid = centroid
            print(last_centroid)
            controller.ControllerInput(-200)
            servo_tick = yaw_controll(controller.u,320)
            print(servo_tick)
            pwm.set_pwm(0, 0, servo_tick)
            cnt += 1
            print(cnt)
        except KeyboardInterrupt:
            cv2.destroyAllWindows() 
            camera.release()
            destroy()
            

                

'''  
