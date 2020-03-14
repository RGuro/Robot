#!/usr/bin/env python


import rospy

class DifferentialDrive:
    def __init__(self, wheelbase, wheel_radius):
        # In meters per radian
        # L is the radius of the circle drawn from turning one wheel while
        # holding the other one still - happens to also be the wheelbase
        self.wheelbase = wheelbase 
        self.wheel_radius = wheel_radius

    def uni_to_diff(self, v, w):
        '''
        Return mm per sec wheel velocities
        '''

        # In meters per radian
        L = self.wheelbase
        R = self.wheel_radius
        
        # w is angular velocity counter clockwise - radians/sec
        # v - m/s
        # L - wheelbase - meter/radian
        # R - wheel radius - meter/radian
        vr = ((2.0 * v) + (w * L)) / (2.0 * R)
        vl = ((2.0 * v) + (-1.0 * w * L)) / (2.0 * R)

        # In m per sec
        return {"vl": vl, "vr": vr}
