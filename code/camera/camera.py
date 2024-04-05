import numpy as np
import cv2
import math
import random

import cv2   #OpenCV를 사용하기위해 import해줍니다.

def main():
    camera = cv2.VideoCapture(-1) #카메라를 비디오 입력으로 사용. -1은 기본설정이라는 뜻
    camera.set(3,640)  #띄울 동영상의 가로사이즈 640픽셀
    camera.set(4,480)  #세로사이즈 480픽셀
    
    while( camera.isOpened() ):  #카메라가 Open되어 있다면,
        _, image = camera.read()
        point_tracking(image)
        
        if cv2.waitKey(1) == ord('q'):  #만약 q라는 키보드값을 읽으면 종료합니다.
            break
        
    cv2.destroyAllWindows() #이후 openCV창을 종료합니다.
    
def point_tracking(image):
    _, binary_image = cv2.threshold(image, 128, 255, cv2.THRESH_BINARY)

    contours, _ = cv2.findContours(binary_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    contours = sorted(contours, key=cv2.contourArea, reverse=True)[:5]  

    longest_contour = max(contours, key=lambda c: cv2.arcLength(c, closed=False))
    longest_contour = max(contours, key=lambda c: cv2.arcLength(c, closed=False))

    mask = np.zeros(image.shape, dtype=np.uint8)

    cv2.drawContours(mask, [longest_contour], -1, color=0, thickness=-1)

    M = cv2.moments(mask)

    if M["m00"] != 0:
        centroid_x = int(M["m10"] / M["m00"])
        centroid_y = int(M["m01"] / M["m00"])
    else:
        centroid_x = mask.shape[1] // 2
        centroid_y = mask.shape[0] // 2

    output_image = image.copy()
    cv2.circle(output_image, (centroid_x, centroid_y), 10, (100, 100, 100), -1)

    output_image = cv2.cvtColor(output_image, cv2.COLOR_BGR2RGB)

    output_image_path = '/mnt/data/road_with_central_point.png'
    cv2.imwrite(output_image_path, output_image)

    cv2.imshow("a", output_image)

    central_coordinates = (centroid_x, centroid_y)
    central_coordinates, output_image_path

if __name__ == '__main__':
    main()
