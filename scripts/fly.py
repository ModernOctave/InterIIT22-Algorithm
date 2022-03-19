#!/usr/bin/env python

import rospy
from mavros_msgs.msg import Thrust
from std_msgs.msg import Header
from mavros_msgs.srv import SetMode

def talker():
    pub = rospy.Publisher('/mavros/setpoint_attitude/thrust', Thrust, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        h = Header()
        h.stamp = rospy.Time.now()
        pub.publish(h,0.5)
        rate.sleep()

def arm():
    rospy.wait_for_service('/mavros/set_mode')
    try:
        set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        set_mode(208, '')
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == '__main__':
    arm()
    try:
        talker()
    except rospy.ROSInterruptException:
        pass