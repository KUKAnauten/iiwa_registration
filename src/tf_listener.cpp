#include <ros/ros.h>
#include <tf/transform_listener.h>


int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_listener");

  ros::NodeHandle node;

  tf::TransformListener listener;

  ros::Rate rate(10.0);
  while (node.ok()){
    tf::StampedTransform transform;
    try{
    listener.waitForTransform("/base_pose", "/tool_link_ee",
                               ros::Time(0), ros::Duration(10.0));
    listener.lookupTransform("/base_pose", "/tool_link_ee", ros::Time(0), transform);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
   ROS_INFO_STREAM("frame_id:"<< transform.frame_id_);
   ROS_INFO_STREAM("child_frame:"<< transform.child_frame_id_);
   ROS_INFO("frame_id:", transform.stamp_);
   
}

  return 0;
};
