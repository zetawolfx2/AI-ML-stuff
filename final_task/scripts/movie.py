#!/usr/bin/env python

import cv2 as cv
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from final_task.srv import Flag, FlagResponse

path = r'/home/zetawolfx2/catkin_ws/src/final_task/scripts/YS.mp4'

cap = cv.VideoCapture(path)

def display(req):
	pub = rospy.Publisher('movie_stream', Image, queue_size = 1)
	bridge = CvBridge()
	if(req.num==1):
		ret, frame = cap.read()
		ros_image = bridge.cv2_to_imgmsg(frame, encoding = "bgr8")
		pub.publish(ros_image)
	return FlagResponse()

rospy.init_node('movie_module', anonymous = True)
s = rospy.Service('movie_stream', Flag, display)
rospy.spin()
