import rospy
import math
import cv2
import numpy as np
from cv_bridge import CvBridge
from interiit21.srv import*
from mavros_msgs.srv import SetMode

class drone_control:
    def move_to_destination(self,x,y,z,yaw,frame_id='fcu_horiz'):
        rospy.wait_for_service('set_position')
        try:
            set_position = rospy.ServiceProxy('set_position', SetPosition)
            resp1 = set_position(x = x, y = y ,z = z, yaw = yaw, frame_id=frame_id)
            rospy.loginfo(resp1.success)
            return resp1.success
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def land(self):
        rospy.wait_for_service('/mavros/set_mode')
        try:
            set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
            resp1 = set_mode(custom_mode='LAND')
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)