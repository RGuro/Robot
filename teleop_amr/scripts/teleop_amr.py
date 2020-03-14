#!/usr/bin/env python

import rospy

from controller.supervisor import Supervisor
    
def main():
    rospy.init_node('teleop_amr', anonymous=False)
    
    supervisor = Supervisor()
    
    rate = rospy.Rate(2)

    while not rospy.is_shutdown():
        supervisor.execute()
        rate.sleep()


if __name__ == '__main__':
    main()
