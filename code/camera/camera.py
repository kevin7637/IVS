import numpy as np
import cv2
import math
import random

alpha = 0.01  
last_centroid = None 

def camera_main():
    global last_centroid
    global camera
    camera = cv2.VideoCapture(0) 
    camera.set(3,640)  
    camera.set(4,480)  
    while True:
        _, image = camera.read()
        centroid = point_tracking(image)
        if centroid:
            last_centroid = centroid
    #camera.release()
    return last_centroid

def point_tracking(image):
    global last_centroid

    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    _, binary_image = cv2.threshold(gray_image, 127, 255, cv2.THRESH_BINARY)

    contours, _ = cv2.findContours(binary_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = sorted(contours, key=cv2.contourArea, reverse=True)[:2] #2  

    for contour in contours:
        M = cv2.moments(contour)
        if M["m00"] != 0:
            centroid_x = int(M["m10"] / M["m00"])
            centroid_y = int(M["m01"] / M["m00"])
            if last_centroid:
                centroid_x = int(last_centroid[0]*(1-alpha) + centroid_x*alpha)
                centroid_y = int(last_centroid[1]*(1-alpha) + centroid_y*alpha)

            output_image = image.copy()
            cv2.circle(output_image, (centroid_x, centroid_y), 10, (100, 100, 100), -1)

            #cv2.imshow("Output", output_image)
            return (centroid_x, centroid_y),output_image  # Return the new centroid

    return last_centroid,image  # Return the last known centroid if no new centroid was calculated
