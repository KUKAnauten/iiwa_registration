#ifndef IIWA_REGISTRATION_BUTTON_LISTENER_H_
#define IIWA_REGISTRATION_BUTTON_LISTENER_H_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <iiwa_msgs/JointPosition.h>

namespace button_listener{

class ButtonListener
{
public:
  ButtonListener();

 virtual void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

 virtual void jointPosCallback(const iiwa_msgs::JointPosition::ConstPtr& msg);

 virtual void buttonEventCallback(const std_msgs::String::ConstPtr& msg);

  geometry_msgs::Pose pose_rec;
  geometry_msgs::Pose output_pose;
  iiwa_msgs::JointQuantity jointPos_rec;


protected:
  ros::NodeHandle n_; 
  ros::Publisher pub_cartPose;
  ros::Publisher pub_jointPos;
  ros::Subscriber sub_pose;
  ros::Subscriber sub_button;
  ros::Subscriber sub_jointPos;
  

};

} // namespace button_listener
#endif // IIWA_REGISTRATION_BUTTON_LISTENER_H_