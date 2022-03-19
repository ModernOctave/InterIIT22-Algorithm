#!/usr/bin/env python

import rospy
from camera import camera
from image_processing import image_processing
from control import drone_control
import cv2 as cv
from simple_pid import PID
pid = PID(1, 0.1, 0.05, setpoint=0)

def d2r(deg):
    return deg*3.14/180

def follow_road(depth_img):
	global ip, dc
	road_angle = ip.get_road_angle(depth_img)
	print("road angle: ", road_angle)
	dc.move_to_destination(7,0,0,d2r(pid(road_angle)))
	rospy.sleep(5)

if __name__ == "__main__":
	rospy.loginfo("Starting autopilot node")
	rospy.init_node('autopilot')
	rospy.loginfo("Autopilot node started")
	ip = image_processing()
	dc = drone_control()
	cam = camera()
	# dc.move_to_destination(0,0,0,0)
	while not rospy.is_shutdown():
		follow_road(cam.get_depth_image())
	rospy.spin()
