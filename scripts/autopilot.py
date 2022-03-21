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
	global ip, dc, cam
	road_angle = ip.get_road_angle(depth_img, cam)
	rospy.loginfo("road angle: "+str(road_angle))
	rospy.loginfo("road depth: "+str(ip.get_road_depth()))
	if abs(320-ip.get_road_center()) > 45:
		rospy.loginfo("Correcting road center")
		dc.move_to_destination(5,(320-ip.get_road_center())/320*20,0,0)
		rospy.sleep(2)
	dc.move_to_destination(5,0,0,d2r(pid(road_angle)))
	rospy.sleep(0.5)
	dc.move_to_destination(5,0,18-ip.get_road_depth(),0)
	rospy.sleep(0.5)

if __name__ == "__main__":
	rospy.loginfo("Starting autopilot node")
	rospy.init_node('autopilot')
	rospy.loginfo("Autopilot node started")
	ip = image_processing()
	dc = drone_control()
	cam = camera()
	dc.move_to_destination(0,0,0,0)
	rospy.sleep(5)
	ip.update_drone_depth(cam.get_depth_image())
	dc.move_to_destination(0,0,18-ip.get_road_depth()+1,0)
	while ip.get_road_depth() < 18:
		ip.update_drone_depth(cam.get_depth_image())
		rospy.loginfo("Gaining elevation")
		rospy.sleep(0.1)
	dc.move_to_destination(5,0,0,0)
	rospy.sleep(3)
	while not rospy.is_shutdown():
		follow_road(cam.get_depth_image())
	rospy.spin()
