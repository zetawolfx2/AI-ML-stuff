#!/usr/bin/env python

import cv2 as cv
import numpy as np
import rospy
from sensor_msgs.msg import Image
from final_task.srv import Flag, FlagResponse
from cv_bridge import CvBridge, CvBridgeError

def adjust_gamma(image, gamma=1.5):
    invGamma = 1.0 / gamma
    table = np.array([((i / 255.0) ** invGamma) * 255
        for i in np.arange(0, 256)]).astype("uint8")
    return cv.LUT(image, table)

path =  r'/home/zetawolfx2/catkin_ws/src/final_task/scripts/Task_Video.mp4'

rospy.init_node('central_priority_node', anonymous = True)

cap = cv.VideoCapture(path)

bridge = CvBridge()

fgbg = cv.createBackgroundSubtractorMOG2(history = 45, varThreshold = 200, detectShadows=True)

rospy.wait_for_service('camera_stream')
camera_stream = rospy.ServiceProxy('camera_stream', Flag)
rospy.wait_for_service('movie_stream')
movie_stream = rospy.ServiceProxy('movie_stream', Flag)

count = 0
flag = 0

prev_priority = 1
priority = 1

def callback(data):
	cv_image = bridge.imgmsg_to_cv2(data, desired_encoding = "bgr8")
	cv.imshow("Stream", cv_image)
	cv.waitKey(1)

rospy.Subscriber('movie_stream', Image, callback)
rospy.Subscriber('camera_stream', Image, callback)

while (cap.isOpened()):
	ret, frame = cap.read()
	copy = np.copy(frame)

	try:
		h, w = frame.shape[:-1]
	except:
		break

	square = frame[h//2-70:h//2+120, w//2-155:w//2+155]

	square = adjust_gamma(square)

	fgmask = fgbg.apply(square)

	ret, fgmask = cv.threshold(fgmask, 30, 255, cv.THRESH_BINARY)
	kernel = np.ones((2, 2), np.uint8)
	fgmask = cv.dilate(fgmask, kernel, iterations=2)

	grey = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

	hs, ws = square.shape[:-1]

	height, width = grey.shape

	blur = cv.GaussianBlur(grey, (5, 5), 1)

	edges = cv.Canny(grey, 150, 250)

	points = np.array([(50, height), (1150, height), (850, 480), (450, 480)])
	mask = np.zeros_like(edges)
	cv.fillPoly(mask, np.int32([points]), 255)
	masked_img = cv.bitwise_and(edges, mask)

	points1 = np.array([(width / 2 + 170, height), (1150, height), (850, 450), (width / 2 + 170, 450)])
	points2 = np.array([(50, height), (width / 2 - 60, height), (width / 2 - 60, 450), (450, 450)])
	mask_left = np.zeros_like(edges)
	cv.fillPoly(mask_left, np.int32([points2]), 255)
	mask_right = np.zeros_like(edges)
	cv.fillPoly(mask_right, np.int32([points1]), 255)
	masked_img_left = cv.bitwise_and(edges, mask_left)
	masked_img_right = cv.bitwise_and(edges, mask_right)

	indices_left = np.where(masked_img_left == [255])
	indices_right = np.where(masked_img_right == [255])

	point1 = (indices_left[1][0], indices_left[0][0])
	point2 = (indices_right[1][0], indices_right[0][0])
	point3 = (indices_left[1][0], 0)
	point4 = (indices_right[1][0], 0)
	point5 = (indices_left[1][0], indices_left[0][0] - 150)
	point6 = (indices_right[1][0], indices_right[0][0] - 200)

	ROI = frame[point6[1]:point2[1], point5[0]:point6[0]]

	indices = np.where(fgmask == [255])

	h_ar, w_ar = ROI.shape[:-1]

	ROI_Area = h_ar*w_ar

	count = count+1

	if(count==60):
		eff = float(ROI_Area/len(indices[0]))
		eff = 1/eff
		prev_priority = priority
		if (eff>0.090):
		    priority = 3
		elif (0.050<eff<0.090):
		    priority = 2
		elif (eff<0.050):
		    priority = 1
		count = 0

	if(priority==1):
		movie_stream(1)
		camera_stream(0)
	elif (priority == 2) and (prev_priority == 1):
		movie_stream(1)
		camera_stream(0)
		flag = 1
	elif (priority == 2) and (prev_priority == 2):
		if (flag == 1):
			movie_stream(0)
			camera_stream(1)
			if (count == 59):
				flag = 0
		elif (flag == 0):
			movie_stream(1)
			camera_stream(0)
			if (count == 59):
				flag = 1
	elif(priority==3):
		movie_stream(0)
		camera_stream(1)

	k = cv.waitKey(9) & 0xff
	if k == 27:
		break

cap.release()
cv.destroyAllWindows()
