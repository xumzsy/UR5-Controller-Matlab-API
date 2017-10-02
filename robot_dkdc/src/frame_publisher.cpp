// Author: Mengze Xu
// Data: 07-24-2017
// This node listens to topic "matlab_frame", updates existing frames and publishes new frames 
// implemented by tf2_ros instead of tf

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/TransformStamped.h>
#include <vector>
#include <string>
class MultiFramePublisher
{
public:
    MultiFramePublisher();

private:
	ros::NodeHandle nh;
	ros::Timer timer; // used to publish frame periodly otherwise RVIZ won't display them
	ros::Subscriber sub; // used to subscribe topic published by matlab
	tf::TransformBroadcaster br; // broadcaster, may need to be static
	std::vector<geometry_msgs::TransformStamped> frame_list; // vector to store frames

	void updateFrameList(const geometry_msgs::TransformStamped::ConstPtr& msg);
	void timeCallback(const ros::TimerEvent&);

};

// constructor
MultiFramePublisher::MultiFramePublisher()
{
	// initialize topics
	sub = nh.subscribe<geometry_msgs::TransformStamped>("/matlab_frame", 10, &MultiFramePublisher::updateFrameList, this);
	timer = nh.createTimer(ros::Duration(0.5), &MultiFramePublisher::timeCallback, this);
}

// timeCallback
void MultiFramePublisher::timeCallback(const ros::TimerEvent&)
{
	if(!frame_list.empty()){
		// if frame list is not empty
		tf::Transform msg;
		for(int i=0;i<frame_list.size();i++){
			// frame_list[i] geometry_msgs::TransformStamped
			msg.setOrigin(tf::Vector3(frame_list[i].transform.translation.x,frame_list[i].transform.translation.y,frame_list[i].transform.translation.z));
			tf::Quaternion q;
 		 	tf::quaternionMsgToTF(frame_list[i].transform.rotation, q);
  			msg.setRotation(q);
			br.sendTransform(tf::StampedTransform(msg,ros::Time::now(),frame_list[i].header.frame_id,frame_list[i].child_frame_id));			
		}
	}
	else{
		ROS_INFO("Warning: frame list is empty");
	}
}

// update
void MultiFramePublisher::updateFrameList(const geometry_msgs::TransformStamped::ConstPtr& msg)
{
	ROS_INFO("Receive Frame Update!");
	std::string msg_name = msg->child_frame_id;
	std::string exist_name;

	for(int i=0;i<frame_list.size();i++){
		exist_name = frame_list[i].child_frame_id;
		if(msg_name.compare(exist_name)==0){
			// existed frames
			if(msg->header.frame_id.compare("Delete")==0){
				ROS_INFO("Delete Frame");
				frame_list.erase(frame_list.begin()+i);
				return;
			}
			else {
				ROS_INFO("Existing frame");
				frame_list[i] = *msg;
				return;
			}
		}
	}
	ROS_INFO("New Frame");
	frame_list.push_back(*msg);
	std::cout<<"list size"<<frame_list.size()<<std::endl;
}

// main function
int main(int argc, char** argv){
	ros::init(argc, argv, "MultiFramePublisher");
	MultiFramePublisher framepub;
	ros::spin();
	return 0;
}
