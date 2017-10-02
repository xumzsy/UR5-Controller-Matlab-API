// Author: Mengze Xu
// Date 07-24-2017
// This node connects ur_modern_drive package and Matlab interface



#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <string>


class ur_interface
{
private:
  // Common Member
  ros::NodeHandle nh;


  // Update joint_states to member current_joint_states
  ros::Subscriber joint_states_sub;
  void joint_states_callback(const sensor_msgs::JointState::ConstPtr& joint);
  
  // Move in Joint space 
  ros::Subscriber trajectory_goal_sub;
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>* action_client;
  void trajectory_goal_callback(const trajectory_msgs::JointTrajectory::ConstPtr& trajectory);
  
  // To get frames
  tf::TransformListener frame_listener;


public:
  ur_interface();
  sensor_msgs::JointState current_joint_states;
  // Get Current Link Frame
  geometry_msgs::PoseStamped get_tf_frame(std::string base, std::string target);

};


// constructor
ur_interface::ur_interface()
{
  joint_states_sub = nh.subscribe<sensor_msgs::JointState>("/joint_states", 10,&ur_interface::joint_states_callback,this);

  trajectory_goal_sub = nh.subscribe<trajectory_msgs::JointTrajectory>("/trajectory_goal",1,&ur_interface::trajectory_goal_callback,this);
  // set up actionlib client
  action_client = new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>("follow_joint_trajectory",true); 

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  action_client->waitForServer(); //will wait for infinite time
  ROS_INFO("Action server started, sending goal.");

}



// Function to get frame
geometry_msgs::PoseStamped ur_interface::get_tf_frame(std::string base, std::string target)
{

  // tf listener
  tf::StampedTransform transform_;
  frame_listener.waitForTransform(base,target,ros::Time(),ros::Duration(2.0));
  try
  { 
     frame_listener.lookupTransform(base, target, ros::Time(0), transform_);  
  }
  catch (tf::TransformException ex)
  {
     ROS_ERROR("%s",ex.what());
  }
   
  // set frame data
  geometry_msgs::PoseStamped frame_;
  frame_.header.stamp = ros::Time::now();
  frame_.header.frame_id = target;
  frame_.pose.position.x = transform_.getOrigin().getX();
  frame_.pose.position.y = transform_.getOrigin().getY();
  frame_.pose.position.z = transform_.getOrigin().getZ();
  frame_.pose.orientation.x = transform_.getRotation().getX();
  frame_.pose.orientation.y = transform_.getRotation().getY();
  frame_.pose.orientation.z = transform_.getRotation().getZ();
  frame_.pose.orientation.w = transform_.getRotation().getW();
  
  return frame_;
}

// update current_joint_states
void ur_interface::joint_states_callback(const sensor_msgs::JointState::ConstPtr& joint)
{
  current_joint_states = *joint;
}

// move in joint space
void ur_interface::trajectory_goal_callback(const trajectory_msgs::JointTrajectory::ConstPtr& trajectory)
{
  control_msgs::FollowJointTrajectoryGoal action_goal;
  action_goal.trajectory = *trajectory;
  action_client->sendGoal(action_goal);
  action_client->waitForResult(ros::Duration(60.0));
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "ur_interface");
  ur_interface ur5;
  ros::spin();
  return 0;
}
  

  
