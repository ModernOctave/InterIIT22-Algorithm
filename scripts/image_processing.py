#!/usr/bin/env python
import sys
import cv2
import numpy as np
import random as rng
rng.seed(12345)

class image_processing:
    def get_centroid(self, contour, shift=(0,0)):
        M = cv2.moments(contour)
        cx, cy = None, None
        if M['m00'] != 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
        else:
            print('Centroid could not be found!')
        return cx+shift[0], cy+shift[1]

    def get_angle(self, centroid1, centroid2):
        if None in centroid1 or None in centroid2:
            return None
        angle = np.arctan2(centroid2[1]-centroid1[1], centroid2[0]-centroid1[0])
        if angle < 0:
            angle = angle + 2*np.pi
        return angle/np.pi*180-90

    def get_contours(self, cv_image, road_height, debug=False):
        margin = 35
        mask = cv2.inRange(cv_image, (road_height[0]-margin, road_height[1]-margin, road_height[2]-margin), (road_height[0]+margin, road_height[1]+margin, road_height[2]+margin))
        if debug:
            cv2.imshow('mask', mask)
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=45)
        if debug:
            cv2.imshow('erroded', mask)
        mask = cv2.dilate(mask, kernel, iterations=10)
        if debug:
            cv2.imshow('dilated', mask)
        mask = cv2.erode(mask, kernel, iterations=15)
        if debug:
            cv2.imshow('erroded2', mask)
        contours, hierarchy = cv2.findContours(mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if debug:
            ri = np.copy(cv_image)
            cv2.drawContours(ri, contours, -1, (0,255,0), 3)
            cv2.imshow('contours', ri)
            cv2.waitKey(0)
        if len(contours) == 0:
            print('No contours found!')
        print(len(contours))
        return contours[0]

    def get_road_angle(self, cv_image, debug=False):
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)
        height, width, channels = cv_image.shape
        road_height = []
        for i in range(channels):
            road_height.append(np.average(cv_image[int(height/2-100):int(height/2+100), int(width/2-20):int(width/2+20),i]))

        contours = self.get_contours(cv_image, road_height)
        contour1 = self.get_contours(cv_image[:240][:], road_height)
        contour2 = self.get_contours(cv_image[240:][:], road_height)
        centroid1 = self.get_centroid(contour1)
        centroid2 = self.get_centroid(contour2,(0,240))
        angle = self.get_angle(centroid1, centroid2)
        print('Angle: ', angle)
        if debug:
            cv_image = cv2.circle(cv_image, centroid1, radius=1, color=(0, 0, 255), thickness=-1)
            cv_image = cv2.circle(cv_image, centroid2, radius=1, color=(0, 0, 255), thickness=-1)
            cv2.drawContours(cv_image, [contours], -1, (0,255,0), 3)
            cv2.imshow('contours', cv_image)
        rect = cv2.minAreaRect(contours[0])
        (x_min, y_min), (w_min, h_min), angle = rect
        if angle > 0:
            angle = angle-90
        else:
            angle = angle+90
        if angle < -45:
            angle = 90 + angle
        elif angle > 45:
            angle = -90 + angle
        cv2.waitKey(0)
        return angle

def main(args):
    ic = image_processing()
    for i in range(43):
        image = cv2.imread(sys.argv[1]+'depth-road-' + str(i) + '.png')
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        ic.get_road_angle(image,True)

if __name__ == '__main__':
    main(sys.argv)