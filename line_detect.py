# -*- coding: utf8 -*-
import cv2
import numpy as np
import time
import socket
import sys
import motor
import servo
import Adafruit_PCA9685
def get_center_line(lines):
    if len(lines) == 0:
        return None
    x1_avg = sum(line[0] for line in lines) / len(lines)
    y1_avg = sum(line[1] for line in lines) / len(lines)
    x2_avg = sum(line[2] for line in lines) / len(lines)
    y2_avg = sum(line[3] for line in lines) / len(lines)
    slope_avg = sum(line[4] for line in lines) / len(lines)
    return [x1_avg, y1_avg, x2_avg, y2_avg, slope_avg]

def DetectLineSlope(src):
    gray = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)
    can = cv2.Canny(gray, 50, 200, None, 3)
    height = can.shape[0]
    rectangle = np.array([[(0, height), (30, 200), (610, 200), (640, height)]])
    mask = np.zeros_like(can)
    cv2.fillPoly(mask, rectangle, 255)
    masked_image = cv2.bitwise_and(can, mask)
    ccan = cv2.cvtColor(masked_image, cv2.COLOR_GRAY2BGR)
    
    roi_points = rectangle
    cv2.polylines(src, [roi_points], True, (0, 255, 255), thickness=2)
    line_arr = cv2.HoughLinesP(masked_image, 1, np.pi / 180, 20, minLineLength=10, maxLineGap=10)
    line_segments = []
    if line_arr is not None:
        for line in line_arr:
            x1, y1, x2, y2 = line[0]
            slope = np.arctan2(y2 - y1, x2 - x1) * 180 / np.pi
            line_segments.append([x1, y1, x2, y2, slope])

    left_lines, right_lines = [], []
    for line in line_segments:
        x1, y1, x2, y2, slope = line
        if slope < 0 and abs(slope) > 20:
            left_lines.append(line)
        elif slope > 0 and abs(slope) > 20:
            right_lines.append(line)

    left_center_line = get_center_line(left_lines)
    right_center_line = get_center_line(right_lines)

    if left_center_line is not None:
        x1, y1, x2, y2, slope = left_center_line
        cv2.line(ccan, (int(x1), int(y1)), (int(x2), int(y2)), (255, 0, 0), 10, cv2.LINE_AA)
        degree_L = slope
    else:
        degree_L = 0

    if right_center_line is not None:
        x1, y1, x2, y2, slope = right_center_line
        cv2.line(ccan, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 10, cv2.LINE_AA)
        degree_R = slope
    else:
        degree_R = 0

    mimg = cv2.addWeighted(src, 1, ccan, 1, 0)
    return mimg, degree_L, degree_R
    
class PIDController:
    def __init__(self, Kp, Ki, Kd, setpoint):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.prev_error = 0
        self.integral = 0

    def update(self, feedback):
        error = self.setpoint - feedback
        self.integral += error
        derivative = error - self.prev_error
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.prev_error = error
        motor.Forward(move_speed)
        return output

#parameter
pid_controller = PIDController(Kp=0.42, Ki=0.0005, Kd=0.25, setpoint=0)
motor.setup()
cap = cv2.VideoCapture(0)
move_speed = 0
motor.Forward(0)
servo.Go()
delay = 0
# Initialize the PCA9685 board
pwm = Adafruit_PCA9685.PCA9685()

# Set PWM frequency (50Hz is typical for servos)
pwm.set_pwm_freq(50)

servo_channel = 0
pwm.set_pwm(servo_channel, 0, 307)
#pwm.set_pwm(0, 0, 100)
#pwm.set_pwm(1, 0, 307)
left_distance = 0
right_distance = 0
x = 200
x2 = 20
while cap.isOpened():
    #motor.Forward(move_speed)
    ret, frame = cap.read()
    if ret:
        frame = cv2.resize(frame, (640, 360))
        cv2.imshow('image',frame)
        cv2.line(frame, (320, 0), (320, 360), (0, 0, 255), 2)
        cv2.imshow('ImageWindow', DetectLineSlope(frame)[0])
        l, r = DetectLineSlope(frame)[1], DetectLineSlope(frame)[2]
        
        if l is None and r is None:
            motor.Stop()
            continue
            
        if l is not None:
            left_distance = l
            if left_distance > x:
                left_distance = x
        else:
            left_distance = x
            
        if r is not None:
            right_distance = r
            if right_distance < -x:
                right_distance = -x
        else:
            right_distance = - x
        
        if left_distance is not None and left_distance < 0:
            left_distance = x2
        elif right_distance is not None and right_distance > 0:
            right_distance = x2
            
        if l is not None and r is not None:                
            feedback = left_distance + right_distance
        elif l is not None:
            feedback = left_distance - x
        elif r is not None:
            feedback = right_distance + x
        
        steering_angle = pid_controller.update(feedback)
        #steering_angle = abs(steering_angle)
        pulse = int((steering_angle * (412 - 200) / 180) + 300) 
        time.sleep(delay)
        pwm.set_pwm(servo_channel, 0, pulse)
        print("===========")
        print("left_distance = ", left_distance)
        print("right_distance = ", right_distance)
        print("feedback =", feedback)
        print("steering_angle = ", steering_angle)
        print("pulse =", pulse)

        key = cv2.waitKey(1)
        if key & 0xFF == ord('q'):
            break
