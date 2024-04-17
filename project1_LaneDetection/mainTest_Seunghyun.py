import argparse
import os
import io
import cv2
import numpy as np
import time
from picamera.array import PiRGBArray
from picamera import PiCamera
import object_detection  
import ultrasonic_sensor  # 초음파 센서 모듈 임포트
import motor_control
import signal  # 시그널 모듈 임포트
from pid_controllerVel import PIDControllerVel  # 앞서 정의한 PIDController 클래스를 가져옵니다.
from pid_controller import PIDController  # 앞서 정의한 PIDController 클래스를 가져옵니다.
from lane_detection import frame_processor
import lateral_control
import trackingmodule






def main():
    default_model_dir = 'all_models'
    default_model = 'ssdlite_mobiledet_coco_qat_postprocess_edgetpu.tflite'
    default_labels = 'coco_labels.txt'
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', help='.tflite model path',
                        default=os.path.join(default_model_dir, default_model))
    parser.add_argument('--labels', help='label file path',
                        default=os.path.join(default_model_dir, default_labels))
    parser.add_argument('--top_k', type=int, default=3,
                        help='number of categories with highest score to display')
    parser.add_argument('--threshold', type=float, default=0.3,
                        help='classifier score threshold')
    args = parser.parse_args()

    # 모델 초기화
    interpreter, labels, inference_size, top_k, threshold = object_detection.initialize_model(args.model, args.labels, args.top_k, args.threshold)

    # PiCamera 초기화
    camera = PiCamera()
    camera.resolution = (640, 480)
    camera.framerate = 30
    rawCapture = PiRGBArray(camera, size=(640, 480))
    resize_w, resize_h = 640, 480


    time.sleep(0.1)  # 카메라 초기화 시간

    # 모터 초기화
    motor_control.setup()

    # 초음파 초기화
    ultrasonic_sensor.init_ultrasonic()

    # Servo 초기화
    lateral_control.lateral_set()

    # Line Sensor 초기화
    trackingmodule.setup()


    pidVel = PIDControllerVel(Kp=1, Ki=0, Kd=0)
    pidSteer = PIDController(Kp=0.05, Ki=0, Kd=0)
    steerAngle = 0
    obstacleFlag = 0 
    laneChangeFlag = 0
    vehicleDetectFlag = 0
    leftPrev = 0
    leftPrevPrev = 0
    stopSignFlag = 0
    vel = 0 
    
    try:
        for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):

            cv2_im = frame.array
    
            # Object Detection
            objs = object_detection.detect_objects(interpreter, labels, inference_size, top_k, threshold, cv2_im)
            cv2_imObj, stopSignSize = object_detection.append_objs_to_img(cv2_im, inference_size, objs, labels)
            # cv2.imshow('frame', cv2_imObj), cv2.waitKey(1)
            for obj in objs:
                if obj.id == 2 and vehicleDetectFlag == 0:
                    vehicleDetectFlag = 1
                    print("vehicle Detected")
                    break

            if stopSignSize > 12000:
                print("StopSign Detected")
                break

            # Lane Detection
            color_frame = cv2.cvtColor(cv2_im, cv2.COLOR_BGR2RGB)
            color_frame = cv2.resize(color_frame, (resize_w, resize_h))
            
            blend_frame, mid_point = frame_processor(color_frame)
            if mid_point == -1:
                diff = None
            else:
                diff = (mid_point - int(resize_w/2))
                
            # print(mid_point, diff)
            cv2.imshow('blend', cv2.cvtColor(blend_frame, cv2.COLOR_RGB2BGR)), cv2.waitKey(1)

            if diff is not None:
                 steerAngle = pidSteer.update(diff,steerAngle,time.time())
            
            
            # ultraSonic Distance
            distance = ultrasonic_sensor.checkdist() * 100
            vel = pidVel.update(distance,vel,time.time())
            if vel <0:
                 motor_control.move(abs(vel), 'backword')  

            control_Vel = -pidVel.update(10, distance, time.time())
            if distance <= 90 and obstacleFlag == 0 and laneChangeFlag == 0 and vehicleDetectFlag == 1:
                obstacleFlag = 1
                print("[Lane Change Start] Vehicle Distance %.2f [cm]" % distance)
  


            if obstacleFlag == 1:
                left, middle, right = trackingmodule.run()
                
                print('left: %d   middle: %d   right: %d\n'%(left,middle,right))
                steerAngle = 2
                if left == 0 and leftPrev == 1 and leftPrevPrev == 1:
                    print('[Lane Change Finish] Lane Sensor Detected')
                    obstacleFlag = 0
                    laneChangeFlag = 1
                    steerAngle = -8
                leftPrevPrev = leftPrev
                leftPrev = left
                    

            motor_control.move(45, 'forward')  
            lateral_control.lateral_control(input_value=steerAngle) # -10 ~ 10

            rawCapture.truncate(0)

    finally:
        # 정리 코드
        cv2.destroyAllWindows()
        camera.close()
        motor_control.destroy()
        ultrasonic_sensor.cleanup()



if __name__ == '__main__':
    main()
