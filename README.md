¿# teleop_amr
steps for enable robot

1.this file enable all devices.

$ cd Enable_robot
$ sudo ./device_permission.bash	                    //password 12345
$ roslaunch rosserial_python Robot_connect.launch

###############################################

2. ---- CONNECT PC to ROBOT -----.

$ cd catkin_ws_robot
$ source devel/setup.bash
$ roslaunch teleop_amr enable_robot.launch         // run all scripts robot sensor and actuadors.
$ roslaunch teleop_amr enable_robot.launch 

------ optional run individual scripts robot sensor ---------

$ rosrun teleop_amr robot_hadware.py               // conversion motor power.
$ rosrun teleop_amr sensor_imu_publisher.py        // IMU PUB/TF.
$ rosrun teleop_amr Sensor_odom_publisher.py	   // odometry PUB/TF.
$ rosrun teleop_amr teleop_amr.py	           // Robot Kinematics.


##############################################


---- teleoperation--mode -----

$ roslaunch teleop_twist_joy teleop.launch


----------------------------


------     Slam         ----------

1. first enable teleoperaion mode

$ cd catkin_ws_robot
$ source devel/setup.bash
$ roslaunch hector_slam_launch tutorial.launch

------------------------------------



---- Autonomous Navigation -------




