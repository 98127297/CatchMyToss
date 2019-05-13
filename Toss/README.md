# All code for use in tossing gets put in here

	Tested: 
		1. ROS direct publishing to ur_driver/joint_speed (doesn't work due to Matlab publishing limit)
		2. ROS actionlib control using Matlab for UR3 (works)

	Usage:
		1. UR3RosControl Class 

Create an object of UR3RosControl to create ROS connection with the master and provide moving to a certain joint angle function and tossing trajectory service call (if available)
