robot_dkdc provides object ur_interface to communicate with ur5
===
1.How to Get Joints
member "current_joint_states" (sensor_msgs::JointStates) provides joint_states.

2.How to move in joint space

2.How to Get frames
geometry_msgs::PoseStamped get_tf_frame(std::string base, std::string target);
return tf_frame from base to target




