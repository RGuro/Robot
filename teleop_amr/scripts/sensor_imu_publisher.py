#!/usr/bin/env python

import time
import rospy
import math
from std_msgs.msg import String
from geometry_msgs.msg import Quaternion
from tf.broadcaster import TransformBroadcaster
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_from_euler


###########################################################################################################################
class RobotImuPublisherNode:
###########################################################################################################################    
    def __init__(self):      
	
	# Initialize the node and name it.
    	rospy.init_node('sensor_imu_publisher')
	self.nodename = rospy.get_name()
	rospy.loginfo("-I- %s started" % self.nodename)
	
	self.degrees2rad = math.pi/180.0

        # Get values from launch file
        self.rate = rospy.get_param('~rate', 80.0)  # the rate at which to publish the transform
        # Static transform between sensor and fixed frame: x, y, z, roll, pitch, yaw
        # <rosparam param="static_transform">[0, 0, 0, 0, 0, 0]</rosparam>
        self.static_transform = rospy.get_param('~static_transform', [0, 0, 0, 0, 0, 0])
        #self.serial_port = rospy.get_param('~serial_port', "/dev/ttyUSB0")
        self.topic_name = rospy.get_param('~topic_name', "/imu")
        self.fixed_frame = rospy.get_param('~fixed_frame', "base_link")
        self.frame_name = rospy.get_param('~frame_name', "imu_frame")
        self.publish_transform = rospy.get_param('~publish_transform', 'false')

        # subscription
        rospy.Subscriber('MPU9250_IMU', String, self.callbackimu)
        
        # Create a publisher for imu message
        self.pub_imu = rospy.Publisher(self.topic_name, Imu, queue_size=1)
        self.odomBroadcaster_imu = TransformBroadcaster()   
        
        self.imu_msg = Imu()
        self.imu_msg.angular_velocity_covariance[0] = -1
        self.imu_msg.linear_acceleration_covariance[0] = -1

        self.current_time = rospy.get_time()
        self.last_time = rospy.get_time()

        self.seq = 0
        self.imu = String("0,0,0,0,0,0,0,0,0,0,0,0,0")
        

        rospy.on_shutdown(self.shutdown_node)
        rate = rospy.Rate(self.rate)

        
        # Main while loop.
        while not rospy.is_shutdown():
            self.current_time = rospy.get_time()
            
            str_imu = str(self.imu)
            if "data:" in str(self.imu):          
                str_imu = str_imu[:len(str_imu)-1]
                str_imu = str_imu.replace('data: "',"")
        
            str_imu = str_imu.replace('data: "',"")    
            str_imu = str_imu.split(",")
            self.data_imu = []
            
            for i in range(len(str_imu)):
                str_im = str_imu[i]
                self.data_imu.append(float(str_im))
            
            if self.publish_transform:
                quaternion = quaternion_from_euler(self.static_transform[3]*self.degrees2rad,
                                                   self.static_transform[4]*self.degrees2rad,
                                                   self.static_transform[5]*self.degrees2rad)

                #send static transformation tf between imu and fixed frame
                self.odomBroadcaster_imu.sendTransform(
                    (self.static_transform[0], self.static_transform[1], self.static_transform[2]),
                    (quaternion[0], quaternion[1], quaternion[2], quaternion[3]),
                    rospy.Time.now(), self.frame_name, self.fixed_frame
                )

            # publish imu message
            self.imu_msg = Imu()
            self.imu_msg.linear_acceleration.x = self.data_imu[10]
            self.imu_msg.linear_acceleration.y = self.data_imu[11]
            self.imu_msg.linear_acceleration.z = self.data_imu[12]

            self.imu_msg.angular_velocity.x = self.data_imu[7]
            self.imu_msg.angular_velocity.y = self.data_imu[8]
            self.imu_msg.angular_velocity.z = self.data_imu[9]

            self.imu_msg.orientation.x = self.data_imu[0]
            self.imu_msg.orientation.y = self.data_imu[1]
            self.imu_msg.orientation.z = self.data_imu[2]
            self.imu_msg.orientation.w = self.data_imu[3]

            self.imu_msg.header.stamp = rospy.Time.now()
            self.imu_msg.header.frame_id = self.frame_name
            self.imu_msg.header.seq = self.seq

            self.pub_imu.publish(self.imu_msg)
            self.seq += 1
            rate.sleep()


    def callbackimu(self, msg):
        imu_data = msg.data
        self.imu = imu_data
     
    def shutdown_node(self):
        rospy.loginfo("Turning off node: robot_imu_publisher")

# Main function.
if __name__ == '__main__':
  
    try:
        obj_temp = RobotImuPublisherNode()
    except rospy.ROSInterruptException:
        pass
