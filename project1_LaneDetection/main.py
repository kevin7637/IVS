import argparse
import os
import io
import cv2
import numpy as np
import time
from picamera.array import PiRGBArray
from picamera import PiCamera
import object_detection  # 수정된 부분: object_detection 모듈 임포트
from lane_detection import frame_processor
from lateral_control import lateral_control, lateral_set


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
    parser.add_argument('--threshold', type=float, default=0.5,
                        help='classifier score threshold')
    args = parser.parse_args()

    # 모델 초기화
    interpreter, labels, inference_size, top_k, threshold = object_detection.initialize_model(args.model, args.labels, args.top_k, args.threshold)

    # PiCamera 초기화
    camera = PiCamera()
    camera.resolution = (640, 480)
    camera.framerate = 30
    rawCapture = PiRGBArray(camera, size=(640, 480))

    time.sleep(0.1)  # 카메라 초기화 시간

    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        cv2_im = frame.array

        objs = object_detection.detect_objects(interpreter, labels, inference_size, top_k, threshold, cv2_im)
        cv2_im = object_detection.append_objs_to_img(cv2_im, inference_size, objs, labels)

        cv2.imshow('frame', cv2_im)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break

        rawCapture.truncate(0)

    cv2.destroyAllWindows()
    camera.close()

if __name__ == '__main__':
    main()
