#!/usr/bin/env python2
# Software License Agreement (BSD License)
#
# Copyright (c) 2020, Cinvestav.SWL
# All rights reserved.


import rospy
import math 
from std_msgs.msg import Float32
from std_msgs.msg import String


global power_wheel_left, power_wheel_right
power_wheel_left = Float32()
power_wheel_right = Float32()

def callbackleft(msg):
    global power_wheel_left
    power_wheel_left = msg.data

def callbackright(msg):
    global power_wheel_right
    power_wheel_right = msg.data    
    
    
if __name__ == '__main__':
   
    rospy.init_node('robot_hadware', anonymous=True)
    rospy.Subscriber('wheel_power_left', Float32, callbackleft) 
    rospy.Subscriber('wheel_power_right', Float32, callbackright) 
    
    pubL = rospy.Publisher('motorR', String, queue_size=10)
    pubR = rospy.Publisher('motorL', String, queue_size=10)
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        act_power_wheel_left = str(power_wheel_left)
        act_power_wheel_right = str(power_wheel_right)
        A_power_wheel_left = act_power_wheel_left.replace("data:","")
        A_power_wheel_right = act_power_wheel_right.replace("data:","")
        L_power_wheel_left = str(int(round(float(A_power_wheel_left))))
        R_power_wheel_right = str(int(round(float(A_power_wheel_right))))
        pubL.publish(L_power_wheel_left)
        pubR.publish(R_power_wheel_right)
        rate.sleep()
