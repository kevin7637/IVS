
# pip install paramiko
# pip install scp

import cv2
import numpy as np
from picamera.array import PiRGBArray
from picamera import PiCamera

import paramiko 
import scp

ssh = paramiko.SSHClient()
ssh.load_system_host_keys()
ssh.connect('192.168.0.5', 22, 'pi', 'raspberry')
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
scp = scp.SCPClient(ssh.get_transport())

camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 30
rawCapture = PiRGBArray(camera, size=(640,480))

lower_yellow = np.array([20, 100, 100])
upper_yellow = np.array([40, 255, 255])

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	scp.get('cam_image.jpg')
	#image = cv2.imread('/home/pi/cam_image.jpg')
	image = cv2.imread('cam_image.jpg')
	hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
	mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

	moments = cv2.moments(mask)
	if moments["m00"] != 0:
		cx = int(moments["m10"] / moments["m00"])
		cy = int(moments["m01"] / moments["m00"])
		cv2.circle(image, (cx, cy), 5, (0, 255, 0), -1)
		print("Yellow center x: {}, y: {}".format(cx,cy))

	cv2.imshow("Frame", image)
	key = cv2.waitKey(1) & 0xFF
	if key == 27:
		break

	rawCapture.truncate(0)

camera.close()
cv2.destroyAllWindows()

