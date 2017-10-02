#!/bin/sh
sudo apt-get install ros-kinetic-moveit
sudo apt-get install ros-kinetic-ros-control
sudo apt-get install ros-kinetic-ros-controllers
sudo apt-get install ros-kinetic-gazebo-ros-control

source /opt/ros/kinetic/setup.bash
export workspace=$HOME/catkin_ws
mkdir -p $workspace/src
cd $workspace/src
git clone https://git.lcsr.jhu.edu/mxu29/robot_dkdc.git
git clone https://github.com/pal-robotics/aruco_ros.git
git clone https://github.com/bosch-ros-pkg/usb_cam.git
git clone https://github.com/ros-industrial/industrial_core.git
git clone -b iron-kinetic-devel https://github.com/iron-ox/ur_modern_driver.git
git clone https://github.com/ros-industrial/universal_robot.git
cd ..
catkin_make --pkg aruco_msgs
catkin_make
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
