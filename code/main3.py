from control.PID import *
from trejectory.trejectory import *
from move.move import*
from Adafruit_PCA9685 import PCA9685
from camera.camera2 import *
import time

HERTZ = 50
FAIL = -1
WIDTH = 0
pwm = PCA9685()
pwm.set_pwm_freq(HERTZ)
setup()
measure = 0
step_time = 0.1
controller = PD_Controller(measure, step_time)
opt = 0
cnt = 0
camera = cv2.VideoCapture(0) 
camera.set(3,640)  
camera.set(4,480)  
servo_tick = 300
servo_tick2 = 300
pwm.set_pwm(0, 0, servo_tick)
pwm.set_pwm(2,0,servo_tick2) #250 오른쪽 봄
speed_set = 0
basic_speed = 20
#camera.release()
Motor_B_EN = 4    
Motor_B_Pin1 = 14 
Motor_B_Pin2 = 15 
Motor_A_EN = 17
Motor_A_Pin1 = 27
Motor_A_Pin2 = 18
line_pin_right = 19
line_pin_middle = 16
line_pin_left = 20

Tr = 11           
Ec = 8  
HERTZ = 50
    
def line_tracking():
    # 라인 트래킹 센서 측정
    status_right = GPIO.input(line_pin_right)
    status_middle = GPIO.input(line_pin_middle)
    status_left = GPIO.input(line_pin_left)
    return status_right, status_middle, status_left

def distance_stop(distance):
    if distance < 0.2:
        motorStop()
        time.sleep(0.1) 
    else:
        speed_set = 20
        move(speed_set, 'forward')
        
def move_robot(servo_angle, speed, duration):
    pwm.set_pwm(0, 0, servo_angle)
    move(speed, 'forward' if speed > 0 else 'backward')
    time.sleep(duration)

def avoid_obstacle():
    distance = detectObstacle()
    start_time = time.time()

    if distance < 0.3:
        motorStop()
        time.sleep(1)

        while True:
            move_robot(300, basic_speed, 0.1)
            distance = detectObstacle()
            if distance >= 0.3:
                move_robot(0, 0, 0.1)
                break

        move_robot(200, basic_speed + 20, 0.1)
        move_robot(400, basic_speed + 20, 2)
        move_robot(200, basic_speed + 20, 5)
        move_robot(200, basic_speed + 20, 2)

  
if __name__ == "__main__":
    while True:
        try:
            #move(speed_set, 'forward')
            distance = detectObstacle()
            ti = time.time()
            if distance < 0.15:
                motorStop()
                time.sleep(1)
                while True:
                    servo_tick = 300
                    pwm.set_pwm(0, 0, servo_tick)
                    time.sleep(0.1)
                    speed_set = basic_speed
                    move(speed_set, 'backward')
                    distance = detectObstacle()
                    if distance >= 0.25:
                        speed_set = 0
                        move(speed_set, 'forward')
                        time.sleep(0.1)
                        break
                servo_tick = 210
                servo_tick2 = 390
                pwm.set_pwm(0, 0, servo_tick)
                prw.set_pwm(2,0,servo_tick2)
                time.sleep(0.1)
                speed_set = basic_speed
                move(speed_set, 'forward')
                time.sleep(0.1)
                while True:
                    tf = time.time()
                    dt = tf - ti
                    if dt > 3:
                        speed_set = 0
                        move(speed_set, 'forward')
                        time.sleep(0.1)
                        break
            else: 
                speed_set = basic_speed
                move(speed_set, 'forward')
                #좌표 = camera
                _, image = camera.read()
                image = image[260:, :]
                centroid,out_image = point_tracking(image)
                #cv2.imshow("image",out_image)
                #cv2.waitKey(1)
                status_right, status_middle, status_left = line_tracking()
                print(status_right,status_middle,status_left)
                if status_right == 1 or status_middle == 1 or status_left == 1:
                    if centroid:
                        last_centroid = centroid
                        print(last_centroid[0])
                        controller.ControllerInput(last_centroid[0])
                        print(controller.u)
                        if last_centroid[0] > 390:
                            servo_tick = 230
                            servo_tick2 = 350
                            #servo_tick = yaw_controll(controller.u,320)
                        elif last_centroid[0] < 250:
                            servo_tick = 370
                            servo_tick2 = 250
                            #servo_tick = yaw_controll(controller.u,320)
                        else:
                            servo_tick = 300
                            servo_tick2 = 300
                            #servo_tick = yaw_controll(controller.u,320)
                            #print(servo_tick)
                else:
                    if status_right == 0 or (status_right == 0 and status_middle == 0):# 오른쪽이 나가면 왼쪽으로
                        servo_tick = 370
                    elif status_left == 0 or (status_left == 0 and status_middle == 0 ):
                        servo_tick = 230
                    else:
                        move(speed_set, 'backward')
                print(last_centroid[0],servo_tick)
                pwm.set_pwm(0, 0, servo_tick)
                pwm.set_pwm(2,0,servo_tick2)
                time.sleep(0.1)
        except KeyboardInterrupt:
            cv2.destroyAllWindows() 
            camera.release()
            destroy()
            

