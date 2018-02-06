#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/String.h>
#include <tf2/LinearMath/Quaternion.h>
#include <moveit/move_group_interface/move_group_interface.h>

 geometry_msgs::PoseStamped dummy_pose;
 geometry_msgs::TransformStamped static_transformStamped;
 
 void buttonEventCallback(const std_msgs::String::ConstPtr& msg) {
    if(msg->data == "pose_get_pressed") {
      ROS_INFO("Received message: %s", msg->data.c_str());
      dummy_pose = moveit::planning_interface::MoveGroupInterface::getCurrentPose("tool_link_ee");
	
	static tf2_ros::StaticTransformBroadcaster static_broadcaster;	
	  static_transformStamped.header.stamp = ros::Time::now();
	  static_transformStamped.header.frame_id = "world";
	  static_transformStamped.child_frame_id = "base_pose";
	  static_transformStamped.transform.translation.x = dummy_pose.pose.position.x;
	  static_transformStamped.transform.translation.y = dummy_pose.pose.position.y;
	  static_transformStamped.transform.translation.z = dummy_pose.pose.position.z;
	  static_transformStamped.transform.rotation.x = dummy_pose.pose.orientation.x;
	  static_transformStamped.transform.rotation.y = dummy_pose.pose.orientation.y;
	  static_transformStamped.transform.rotation.z = dummy_pose.pose.orientation.z;
	  static_transformStamped.transform.rotation.w = dummy_pose.pose.orientation.w;
	  
	  ROS_INFO("dummy_position = (%f, %f, %f), dummy_orientation (%f, %f, %f. %f)", dummy_pose.pose.position.x, dummy_pose.pose.position.y, dummy_pose.pose.position.z, dummy_pose.pose.orientation.x, dummy_pose.pose.orientation.y, dummy_pose.pose.orientation.z, dummy_pose.pose.orientation.w);
      
       	static_broadcaster.sendTransform(static_transformStamped);
  	ROS_INFO("Spinning until killed publishing %s to world", "base_pose");
      }
}


int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_static_publisher");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/iiwa/state/buttonEvent", 100, &buttonEventCallback);
  
  ros::spin();
  
  return 0;
}
