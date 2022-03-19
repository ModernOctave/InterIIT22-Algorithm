#!/usr/bin/env python
import sys
import cv2
import numpy as np
import random as rng
rng.seed(12345)

class image_processing:
    def get_road_angle(self, cv_image, debug=False):
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)
        height, width, channels = cv_image.shape
        road_height = []
        for i in range(channels):
            road_height.append(np.average(cv_image[int(height/2-100):int(height/2+100), int(width/2-20):int(width/2+20),i]))
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
            cv2.drawContours(cv_image, contours, -1, (0,255,0), 3)
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