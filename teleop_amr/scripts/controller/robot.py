#!/usr/bin/env python

#
#
#
#
#

import rospy
from std_msgs.msg import Float32

class Robot:
    def __init__(self):
        # Diff drive robot attributes can be stored in parameter server
        # wheelbase of 630mm and wheel diameter of 127mm

        self.wheelbase = rospy.get_param("wheelbase", default=0.630)
        self.wheel_radius = rospy.get_param("wheel_radius", default=.0635)

        # Wheel min and max no-load velocities in radians per sec
        self.wheel_speed_min = rospy.get_param("wheel_speed/min", default=0.9422)
        self.wheel_speed_mid = rospy.get_param("wheel_speed/mid", default=18.2582)
        self.wheel_speed_max = rospy.get_param("wheel_speed/max", default=37.4598)
        self.wheel_speed_min_power = \
            rospy.get_param("wheel_speed/min_power", default=250.0)
        self.wheel_speed_mid_power = \
            rospy.get_param("wheel_speed/mid_power", default=2048.0)
        self.wheel_speed_max_power = \
            rospy.get_param("wheel_speed/max_power", default=4095.0)

        # Publish out wheel power
        self.cur_wheel_power_right = Float32()
        self.cur_wheel_power_left = Float32()
        self.cur_wheel_power_right.data = 0.0
        self.cur_wheel_power_left.data = 0.0
        
        self.pub_power_right = \
            rospy.Publisher('/wheel_power_right', Float32, queue_size=10)
        self.pub_power_left = \
            rospy.Publisher('/wheel_power_left', Float32, queue_size=10)
        self.pub_power_right.publish(self.cur_wheel_power_right)
        self.pub_power_left.publish(self.cur_wheel_power_left)

        
    def shutdown(self):
        rospy.loginfo(rospy.get_caller_id() + " Robot shutdown")
        self.cur_wheel_power_right.data = 0.0
        self.cur_wheel_power_left.data = 0.0
        self.pub_power_right.publish(self.cur_wheel_power_right)
        self.pub_power_left.publish(self.cur_wheel_power_left)
        

    def velocity_to_power(self, v):
        av = abs(v)
        # If velocity is below minimum velocity turnable by DAC12bits, then
        # just set to zero since the wheels won't spin anyway
        if av < self.wheel_speed_min:
            return 0.0

        a = 0.0
        b = 0.0
        a_pow = 0.0 
        b_pow = 0.0
        nnn = 0.0
       

        if av >= self.wheel_speed_min and av < self.wheel_speed_mid:
            a = self.wheel_speed_min
            a_pow = self.wheel_speed_min_power
            b = self.wheel_speed_mid
            b_pow = self.wheel_speed_mid_power
        elif av >= self.wheel_speed_mid and av <= self.wheel_speed_max:
            a = self.wheel_speed_mid
            a_pow = self.wheel_speed_mid_power
            b = self.wheel_speed_max    
            b_pow = self.wheel_speed_max_power

        nnn = ((av-a))/(b-a)
        wheel_power = ((nnn*(b_pow - a_pow))+ a_pow) 
        
        # negative for backward      
        if v < 0:
            wheel_power *= -1.0

        return wheel_power

    def set_wheel_speed(self, vr, vl):
        # Clamp the wheel speeds to actuator limits.
        vr = max(min(vr, self.wheel_speed_max), self.wheel_speed_max * -1.0)
        vl = max(min(vl, self.wheel_speed_max), self.wheel_speed_max * -1.0)

        # Convert to power norms
        self.cur_wheel_power_right.data = self.velocity_to_power(vr)
        self.cur_wheel_power_left.data = self.velocity_to_power(vl)

         # Publish out
        if self.cur_wheel_power_right.data != 0.0 or \
           self.cur_wheel_power_left.data != 0.0:
            rospy.loginfo(rospy.get_caller_id() +
                          " right power: " + str(self.cur_wheel_power_right) +
                          " left power: " + str(self.cur_wheel_power_left))
        self.pub_power_right.publish(self.cur_wheel_power_right)
        self.pub_power_left.publish(self.cur_wheel_power_left)
        
        

    
        

        
