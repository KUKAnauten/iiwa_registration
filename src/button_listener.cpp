#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <iiwa_msgs/JointPosition.h>

// Publishes the current CartesianPose and JointPositions of the robot, at the moment the toolbarbutton is pressed

#define BUTTONNAME pose_get


class ButtonListener
{
public:
  ButtonListener()
  {
    
    //Topic you want to subscribe
    sub_pose = n_.subscribe("/iiwa/state/CartesianPose", 1, &ButtonListener::poseCallback, this);

    sub_jointPos = n_.subscribe("/iiwa/state/JointPosition", 1, &ButtonListener::jointPosCallback, this);


    sub_button = n_.subscribe("/iiwa/state/buttonEvent", 1, &ButtonListener::buttonEventCallback, this);

     //Topic you want to publish
    pub_cartPose = n_.advertise<geometry_msgs::Pose>("/iiwa/registered/CartesianPose", 1);

    pub_jointPos = n_.advertise<iiwa_msgs::JointQuantity>("/iiwa/registered/JointPositions", 1);

  }

  // Callbackfunctions
  
  void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
    pose_rec = msg->pose;
  }

  void jointPosCallback(const iiwa_msgs::JointPosition::ConstPtr& msg)
  {
    jointPos_rec = msg->position;
  }

  void buttonEventCallback(const std_msgs::String::ConstPtr& msg)
  {
    geometry_msgs::Pose updated_pose;
    iiwa_msgs::JointQuantity updated_jointPos;

    // get new pose every time the toolbar button is pushed
    if(msg->data == "pose_get_pressed"){ 
        updated_pose = pose_rec;
        updated_jointPos = jointPos_rec;

        ROS_INFO("new_x_pos = %f", updated_pose.position.x);

        pub_cartPose.publish(updated_pose);
        pub_jointPos.publish(updated_jointPos);
    }
    
  }

private:
  ros::NodeHandle n_; 
  ros::Publisher pub_cartPose;
  ros::Publisher pub_jointPos;
  ros::Subscriber sub_pose;
  ros::Subscriber sub_button;
  ros::Subscriber sub_jointPos;
  geometry_msgs::Pose pose_rec;
  geometry_msgs::Pose output_pose;
  iiwa_msgs::JointQuantity jointPos_rec;

};

int main(int argc, char **argv)
{

ros::init(argc, argv , "button_listener_test");

ButtonListener listener;

ros::spin();
  
return 0;
}