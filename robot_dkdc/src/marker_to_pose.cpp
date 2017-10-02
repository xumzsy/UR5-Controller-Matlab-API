// Author: Mengze Xu
// Date: 07-24-2017
// This node subscribes to aruco and publish the message in standard ros message to be received in Matlab
// marker_id is published seperately but with the same order




#include "ros/ros.h"
#include <aruco_msgs/MarkerArray.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseArray.h>
#include "std_msgs/String.h"

class Marker2Pose
{
public: 
  Marker2Pose();

private:
  ros::NodeHandle nh;
  ros::Subscriber marker_sub;
  ros::Publisher pose_pub;
  void markerCallback(const aruco_msgs::MarkerArray::ConstPtr& msg);
};

Marker2Pose::Marker2Pose()
{
  marker_sub = nh.subscribe<aruco_msgs::MarkerArray>("/aruco_marker_publisher/markers", 10, &Marker2Pose::markerCallback, this);
  pose_pub = nh.advertise<geometry_msgs::PoseArray>("/aruco_marker_publisher/PoseArray", 10);
}  

void Marker2Pose::markerCallback(const aruco_msgs::MarkerArray::ConstPtr& msg)
{
  geometry_msgs::PoseArray poseArray;
  poseArray.header.stamp = ros::Time::now();
  for(int i=0;i<msg->markers.size();i++)
  {
    poseArray.poses.push_back(msg->markers[i].pose.pose);
  }
  pose_pub.publish(poseArray);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "readmarker");
  Marker2Pose marker2pose;
  ros::spin();
  
  return 0;
}
