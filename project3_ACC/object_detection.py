import os
import cv2
import numpy as np
from picamera.array import PiRGBArray
from picamera import PiCamera
from pycoral.adapters.common import input_size
from pycoral.adapters.detect import get_objects
from pycoral.utils.dataset import read_label_file
from pycoral.utils.edgetpu import make_interpreter
from pycoral.utils.edgetpu import run_inference

def initialize_model(model_path, label_path, top_k=3, threshold=0.5):
    interpreter = make_interpreter(model_path)
    interpreter.allocate_tensors()
    labels = read_label_file(label_path)
    inference_size = input_size(interpreter)
    return interpreter, labels, inference_size, top_k, threshold


def detect_objects(interpreter, labels, inference_size, top_k, threshold, cv2_im):
    cv2_im_rgb = cv2.cvtColor(cv2_im, cv2.COLOR_BGR2RGB)
    cv2_im_rgb = cv2.resize(cv2_im_rgb, inference_size)
    run_inference(interpreter, cv2_im_rgb.tobytes())
    objs = get_objects(interpreter, threshold)[:top_k]
    return objs

def append_objs_to_img(cv2_im, inference_size, objs, labels):
    stopSignSize = 0
    height, width, channels = cv2_im.shape
    scale_x, scale_y = width / inference_size[0], height / inference_size[1]
    for obj in objs:
        if obj.id in [2, 12]: # 0 person 2 car 12 stopSign
            bbox = obj.bbox.scale(scale_x, scale_y)
            x0, y0 = int(bbox.xmin), int(bbox.ymin)
            x1, y1 = int(bbox.xmax), int(bbox.ymax)

            percent = int(100 * obj.score)
            label = '{}% {}'.format(percent, labels.get(obj.id, obj.id))

            cv2_im = cv2.rectangle(cv2_im, (x0, y0), (x1, y1), (0, 255, 0), 2)
            cv2_im = cv2.putText(cv2_im, label, (x0, y0 + 30),
                                 cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0), 2)
            if obj.id == 12:
                stopSignSize = abs(x0 - x1) * abs(y0 - y1)
    return cv2_im, stopSignSize