#!/usr/bin/env python

#
#
#
#
#

import rospy

from robot import Robot
from rc_teleop import RCTeleop
from dynamics.differential_drive import DifferentialDrive

class Supervisor:
    def __init__(self):
        rospy.on_shutdown(self.shutdown_cb)

        self.controllers = {"rc": RCTeleop()}
        self.current_state = "rc"
        self.current_controller = self.controllers[self.current_state]

        self.robot = Robot()
        rospy.loginfo(rospy.get_caller_id() +
                      " wheelbase: " + str(self.robot.wheelbase) +
                      " wheel radius: " + str(self.robot.wheel_radius))
        
        self.dd = DifferentialDrive(self.robot.wheelbase,
                                    self.robot.wheel_radius)

    def execute(self):
        # Get commands in unicycle model
        ctrl_output = self.current_controller.execute()

        # Convert unicycle model commands to differential drive model
        diff_output = self.dd.uni_to_diff(ctrl_output["v"], ctrl_output["w"])

        if ctrl_output["v"] != 0.0 or ctrl_output["w"] != 0.0:
            rospy.loginfo(rospy.get_caller_id() + " v: " +
                          str(ctrl_output["v"]) +
                          " w: " + str(ctrl_output["w"]))
            rospy.loginfo(rospy.get_caller_id() + " vl: " +
                          str(diff_output["vl"]) +
                          " vr: " + str(diff_output["vr"]))
        
        # Set the wheel speeds
        self.robot.set_wheel_speed(diff_output["vr"], diff_output["vl"])
        
    def shutdown_cb(self):
        for ctrl in self.controllers.values():
            ctrl.shutdown()

        self.robot.shutdown()

        
