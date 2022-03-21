#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
from time import sleep


class camera:

	def __init__(self):
		self.bridge = CvBridge()
		self.rgb_img = None
		self.depth_img = None
		rospy.Subscriber("/depth_camera/rgb/image_raw", Image, self.update_rgb_img)
		rospy.Subscriber("/depth_camera/depth/image_raw", Image, self.update_depth_img)
		self.wait_for_img()
		# fourcc = cv2.VideoWriter_fourcc(*'MP4V') 
		# width, height = 640, 480
		# self.rbg_video = cv2.VideoWriter('/home/om/Videos/rgb_video.mp4', fourcc, 20, (width, height))
		# self.depth_video = cv2.VideoWriter('/home/om/Videos/depth_video.mp4', fourcc, 20, (width, height))

	def update_rgb_img(self, data):
			try:
				cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
			except CvBridgeError as e:
				print(e)
			self.rgb_img = cv_image

	def update_depth_img(self, data):
			try:
				cv_image = self.bridge.imgmsg_to_cv2(data, "passthrough")
			except CvBridgeError as e:
				print(e)
			
			# cv_image = cv_image/30*255
			# cv_image = cv2.normalize(src=cv_image, dst=None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
			# cv_image = cv_image.astype(np.uint8)

			self.depth_img = cv_image

	def get_rgb_image(self):
		return self.rgb_img

	def get_depth_image(self):
		return self.depth_img

	def get_norm_depth_image(self):
		return cv2.normalize(src=-1*self.depth_img, dst=None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)

	def wait_for_img(self):
		while self.rgb_img is None or self.depth_img is None:
			sleep(0.1)
			print("waiting for image")


if __name__ == '__main__':
	rospy.init_node('camera')
	cam = camera()
	while not rospy.is_shutdown():
		# cv2.imshow("Raw Depth Image", cam.get_depth_image())
		cv2.imshow("Depth Image", cam.get_norm_depth_image())
		cv2.imshow("RGB Image", cam.get_rgb_image())
		# print(cam.get_depth())
		cv2.waitKey(1)