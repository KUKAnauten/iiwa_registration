#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <cstdio>
#include <tf2/LinearMath/Quaternion.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_static_publisher");
  ros::NodeHandle n;

  ros::Rate r(10000);
  
  static tf2_ros::StaticTransformBroadcaster static_broadcaster;
  geometry_msgs::TransformStamped static_transformStamped;

  static_transformStamped.header.stamp = ros::Time::now();
  static_transformStamped.header.frame_id = "world";
  static_transformStamped.child_frame_id = "base_pose";
  static_transformStamped.transform.translation.x = 0.530517;
  static_transformStamped.transform.translation.y = -0.181726;
  static_transformStamped.transform.translation.z = 1.16903;
  tf2::Quaternion quat;
  quat.setRPY(atof(argv[5]), atof(argv[6]), atof(argv[7]));
  static_transformStamped.transform.rotation.x = 0.00964753;
  static_transformStamped.transform.rotation.y = 0.611417;
  static_transformStamped.transform.rotation.z = -0.01133;
  static_transformStamped.transform.rotation.w = 0.791169;
  static_broadcaster.sendTransform(static_transformStamped);
  ROS_INFO("Spinning until killed publishing %s to world", "base_pose");
  ros::spin();
  
  return 0;
}
