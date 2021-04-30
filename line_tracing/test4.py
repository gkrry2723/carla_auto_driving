#!/usr/bin/env python
# license removed for brevity

import rospy
import cv2
import cv2 as cv
import sys
import math
import numpy as np
import glob
import os
from cv_bridge import CvBridge

import carla
import time

from std_msgs.msg import String
from sensor_msgs.msg import Image
from carla_msgs.msg import CarlaEgoVehicleControl

bridge = CvBridge()

steer = 0

# 1. subscriber ( make line )

def draw_lines(img, lines, color=[255, 0, 0], thickness=2): #draw line
    #for line in lines:
    x1 = lines[0][0] 
    y1 = lines[0][1] 
    x2 = lines[0][2] 
    y2 = lines[0][3] 
    cv2.line(img, (x1, y1), (x2, y2), color, thickness)

def weighted_img(img, initial_img, a=1, b=1., c=0.): 
    return cv2.addWeighted(initial_img, a, img, b, c)

def get_fitline(img, f_lines): #    
    lines = np.squeeze(f_lines)
    lines = lines.reshape(lines.shape[0]*2,2)
    rows,cols = img.shape[:2]
    output = cv2.fitLine(lines,cv2.DIST_L2,0, 0.01, 0.01)
    vx, vy, x, y = output[0], output[1], output[2], output[3]
    x1, y1 = int(((img.shape[0]-1)-y)/vy*vx + x) , img.shape[0]-1
    x2, y2 = int(((img.shape[0]/2+100)-y)/vy*vx + x) , int(img.shape[0]/2+100)
    
    result = [x1,y1,x2,y2]
    return result

def draw_fit_line(img, lines, color=[255, 0, 0], thickness=10): #
        cv2.line(img, (lines[0], lines[1]), (lines[2], lines[3]), [0,0,255], 5)



def callback(data):
	src = bridge.imgmsg_to_cv2(data,desired_encoding='bgr8')
	cdst = bridge.imgmsg_to_cv2(data,desired_encoding='rgb8')
	#src = cv2.resize(src,(480,150))
	#cdst = cv2.resize(src,(480,150))
	src = src[400:600][:].copy()
	cdst = cdst[400:600][:].copy()

	dst = cv.Canny(src,50,200,None,3)
	#cdst = cv.cvtColor(dst, cv.COLOR_BGR2RGB)
	cdstP = np.copy(cdst) 

	linesP = cv.HoughLinesP(dst, 1, np.pi / 180, 50, None, 50, 10)
	
	line_arr = np.squeeze(linesP)

	print(dst.shape)

	#calculate the slope
	slope_degree = (np.arctan2(line_arr[:,1] - line_arr[:,3], line_arr[:,0] - line_arr[:,2]) * 180) / np.pi

	#horizontal slope limit setting
	line_arr = line_arr[np.abs(slope_degree)<160]
	slope_degree = slope_degree[np.abs(slope_degree)<160]

	#vertical slope limit setting
	line_arr = line_arr[np.abs(slope_degree)>95]
	slope_degree = slope_degree[np.abs(slope_degree)>95]

	#filtering line via limit
	L_lines, R_lines = line_arr[(slope_degree>0),:], line_arr[(slope_degree<0),:]
	temp = np.zeros((src.shape[0], src.shape[1], 3), dtype=np.uint8)
	L_lines, R_lines = L_lines[:,None], R_lines[:,None]
	
	# 
	left_fit_line = get_fitline(src,L_lines)
	right_fit_line = get_fitline(src,R_lines)
	# 
	draw_fit_line(temp, left_fit_line)
	draw_fit_line(temp, right_fit_line)

	
	# calculate centor position 
		

#	if len(L_lines) != 0 and len(R_lines) != 0: 
#		L_avg = L_lines.sum(axis=0)/len(L_lines)
#		R_avg = R_lines.sum(axis=0)/len(R_lines)
#		print(L_avg)
#		print(R_avg)
#		center = (L_avg + R_avg) / 2
#		print(center)
#		center_x = center[0][0] + center[0][2] /2
#		
#		# if center on right, go left
#		global steer
#
#		print center_x
#
#		if center_x > 400:
#			steer = -0.3
#		elif center_x < 400:
#			steer = 0.3
#		else:
#			steer=0
#
#	else:
#		steer = 0
#	print steer
	
	
	#draw line
	#draw_lines(temp, L_avg)
	#draw_lines(temp, R_avg)

	result = weighted_img(temp, src) #overlap
	cv2.imshow('result',result) #print
	cv2.waitKey(0)
	cv2.destroyAllWindows() 

rospy.init_node('my_node')
rospy.Subscriber('/carla/ego_vehicle/rgb_front/image', Image, callback, queue_size=1)
pub = rospy.Publisher('/carla/ego_vehicle/vehicle_control_cmd_manual', CarlaEgoVehicleControl, queue_size = 1)
pub_msg = CarlaEgoVehicleControl()
rate = rospy.Rate(13)

while not rospy.is_shutdown():

	#global steer
	pub_msg.throttle = 1
	pub_msg.steer = 0
	pub.publish(pub_msg)
	#rate.sleep()
	pub_msg.throttle = 1
	pub_msg.steer = steer
	pub.publish(pub_msg)
	#rate.sleep()
	pub_msg.throttle = 1
	pub_msg.steer = -steer
	pub.publish(pub_msg)
	rate.sleep()
	
	
	
	
	

rospy.spin()
