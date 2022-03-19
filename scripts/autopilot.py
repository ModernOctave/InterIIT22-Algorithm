#!/usr/bin/env python

import rospy
from camera import camera
from image_processing import image_processing
from control import drone_control
import cv2 as cv
from simple_pid import PID

pid = PID(1, 0.1, 0.05, setpoint=0)
depth_pid = PID(1, 0.1, 0.05, setpoint=18)

def d2r(deg):
    return deg*3.14/180

def follow_road(depth_img):
	global ip, dc, cam
	road_angle = ip.get_road_angle(depth_img, cam)
	print("road angle: ", road_angle)
	dc.move_to_destination(5,0,0,d2r(pid(road_angle)))
	rospy.sleep(0.2)
	dc.move_to_destination(5,0,18-cam.get_depth(),0)
	rospy.sleep(0.2)

if __name__ == "__main__":
	rospy.loginfo("Starting autopilot node")
	rospy.init_node('autopilot')
	rospy.loginfo("Autopilot node started")
	ip = image_processing()
	dc = drone_control()
	cam = camera()
	dc.move_to_destination(0,0,18,0)
	while cam.get_depth < 18:
		pass
	dc.move_to_destination(5,0,0,0)
	rospy.sleep(3)
	while not rospy.is_shutdown():
		follow_road(cam.get_depth_image())
	rospy.spin()
