#include <iiwa_registration/button_listener.h>

// Publishes the current CartesianPose and JointPositions of the robot, at the moment the toolbarbutton is pressed


namespace button_listener{
    
  ButtonListener::ButtonListener(){
    sub_pose = n_.subscribe("/iiwa/state/CartesianPose", 1, &ButtonListener::poseCallback, this);
    sub_jointPos = n_.subscribe("/iiwa/state/JointPosition", 1, &ButtonListener::jointPosCallback, this);
    sub_button = n_.subscribe("/iiwa/state/buttonEvent", 1, &ButtonListener::buttonEventCallback, this);
    pub_cartPose = n_.advertise<geometry_msgs::Pose>("/iiwa/registered/CartesianPose", 1);
    pub_jointPos = n_.advertise<iiwa_msgs::JointQuantity>("/iiwa/registered/JointPositions", 1);
  }

  // Callbackfunctions
  
  void ButtonListener::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
    pose_rec = msg->pose;
  }

  void ButtonListener::jointPosCallback(const iiwa_msgs::JointPosition::ConstPtr& msg)
  {
    jointPos_rec = msg->position;
  }

  void ButtonListener::buttonEventCallback(const std_msgs::String::ConstPtr& msg)
  {
    geometry_msgs::Pose updated_pose;
    iiwa_msgs::JointQuantity updated_jointPos;

    // get new pose every time the toolbar button is pushed
    if(msg->data == "pose_get_pressed"){ 
        updated_pose = pose_rec;
        updated_jointPos = jointPos_rec;

        ROS_INFO("I registered a new pose!");

        pub_cartPose.publish(updated_pose);
        pub_jointPos.publish(updated_jointPos);
    }
    
  }
  } // Namespace


int main(int argc, char **argv)
{

ros::init(argc, argv , "button_listener_test");

button_listener::ButtonListener listener;

ros::spin();
  
return 0;
}
