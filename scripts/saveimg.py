#!/usr/bin/env python
import sys
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
from time import sleep

class listeners:

    def __init__(self):
        self.bridge = CvBridge()
        self.rgb_img = None
        self.depth_img = None
        self.num_img = 0

        rospy.init_node('saveimg')
        rospy.Subscriber("/depth_camera/rgb/image_raw", Image, self.get_rgb_img)
        rospy.Subscriber("/depth_camera/depth/image_raw", Image, self.get_depth_img)

    def get_rgb_img(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        self.rgb_img = cv_image   

    #   cv2.imshow("Depth Image", self.depth_img)
    #   cv2.imshow("RGB Image", self.rgb_img)
    #   cv2.waitKey(1)

    def get_depth_img(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "passthrough")
        except CvBridgeError as e:
            print(e)

        cv_image = -1*cv_image
        cv_image = cv2.normalize(src=cv_image, dst=None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        cv_image = cv_image.astype(np.uint8)

        self.depth_img = cv_image

        # cv2.imshow("Depth Image", cv_image)
        # cv2.imshow("RGB Image", cv_image)
        # cv2.waitKey(1)
        
    def save_img(self):
        if self.rgb_img is not None and self.depth_img is not None:
            cv2.imwrite('/home/om/Desktop/rgb-'+sys.argv[1]+'-'+str(self.num_img)+'.png', self.rgb_img)
            cv2.imwrite('/home/om/Desktop/depth-'+sys.argv[1]+'-'+str(self.num_img)+'.png', self.depth_img)
            print("Image saved")
            self.num_img += 1


if __name__ == '__main__':
    print(sys.argv)
    if len(sys.argv) != 2:
        print("Usage: python saveimg.py <set-name>")
        exit()

    listener = listeners()
    while not rospy.is_shutdown():
        i = input("Press Enter to capture image")
        listener.save_img()