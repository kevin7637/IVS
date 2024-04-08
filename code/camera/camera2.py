import numpy as np
import cv2
from collections import deque

history_length = 10
centroid_history = deque(maxlen=history_length)

def camera_main():
    camera = cv2.VideoCapture(0)
    camera.set(3, 640)
    camera.set(4, 480)

    while(camera.isOpened()):
        _, image = camera.read()
        centroid = point_tracking(image)

        if centroid:
            centroid_history.append(centroid)

        if cv2.waitKey(1) == ord('q'):
            break

    cv2.destroyAllWindows()
    camera.release()

def point_tracking(image):
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    _, binary_image = cv2.threshold(gray_image, 127, 255, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(binary_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    contours = sorted(contours, key=cv2.contourArea, reverse=True)[:5]

    for contour in contours:
        M = cv2.moments(contour)
        if M["m00"] != 0:
            centroid_x = int(M["m10"] / M["m00"])
            centroid_y = int(M["m01"] / M["m00"])

            if centroid_history:
                sum_x = sum_y = 0
                for (hist_x, hist_y) in centroid_history:
                    sum_x += hist_x
                    sum_y += hist_y
                avg_x = sum_x / len(centroid_history)
                avg_y = sum_y / len(centroid_history)
                # 평균과 새로운 중심점을 결합
                centroid_x = int((avg_x + centroid_x) / 2)
                centroid_y = int((avg_y + centroid_y) / 2)

            output_image = image.copy()
            cv2.circle(output_image, (centroid_x, centroid_y), 10, (100, 100, 100), -1)
            #cv2.imshow("Output", output_image)

            return (centroid_x, centroid_y) , output_image

    return centroid_history,image 
'''
if __name__ == '__main__':
    main()
'''
    