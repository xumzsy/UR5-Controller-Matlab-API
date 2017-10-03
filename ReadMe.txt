# UR5-Controller-Matlab-API
This package provides Matlab API to ur_modern_driver which controls the UR5 on ROS. Ideally, this package hides all the ROS things behind and user only needs Matlab skills.

# Requirement
Ubuntu 16.04, ROS Kinetic, Ethernet connection to UR5, recommend to use IP address as 172.22.22.1/2. A detailed instruction can be found here 
https://github.com/qian256/ur5_setup

# How to install
Directly run "./install.sh". Most packages are open source ROS packages and a custom package robot_dkdc is used. 

# How to use
Open new terminal, use "roslaunch robot_dkdc ur_simulation.launch" to start simulation and use "roslaunch robot_dkdc ur_interface.launch" to start real robot.

Open Matlab, create class ur5_interface. Common functions are provides inside this class. Another class tf_frame is provides to visualize static frame in RVIZ.

Note that ur_interface also provide function read_markers to read Aruco markers.

Note that open_gripper is designed for EU-20 gripper by tool digital out. Modify it if other grippers are used.
 
# Credits
This function is originally used in a course lab. Long Qian help a lot and he also provided another method to control UR5 from Matlab. See here
https://github.com/qian256/ur5_setup
