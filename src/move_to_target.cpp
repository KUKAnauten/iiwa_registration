  /*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the author nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Laura Bielenberg */

#include <iimoveit/robot_interface.h>
#include <iiwa_registration/button_listener.h>

namespace move_to_target{

  namespace rvt = rviz_visual_tools;


class TargMover : public iimoveit::RobotInterface {
public:
 TargMover(ros::NodeHandle* node_handle, const std::string& planning_group, const std::string& base_frame)
      : RobotInterface(node_handle, planning_group, base_frame) {
    base_pose_.position.x = 0.5;
    base_pose_.position.y = 0.0;
    base_pose_.position.z = 0.6;
    base_pose_.orientation.x = 0.0;
    base_pose_.orientation.y = 1.0;
    base_pose_.orientation.z = 0.0;
    base_pose_.orientation.w = 0.0;
  }

  void buttonEventCallback(const std_msgs::String::ConstPtr& msg) {
    if(msg->data == "pose_get_pressed") {
      ROS_INFO("Der Schmartie funktioniert: %s", msg->data.c_str());
      current_pose = getPose();
      ROS_INFO("Current Position = (%f, %f, %f)", current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z);
      visual_tools_.publishText(text_pose_, "Pose registered!", rvt::WHITE, rvt::XLARGE);
    }
  }

  void moveToBasePose() {
    planAndMove(base_pose_, std::string("base pose"));
    visual_tools_.publishText(text_pose_, "I'm at my base pose...press NEXT to get to the registered pose", rvt::WHITE, rvt::XLARGE);
  }

  void moveToRegisteredPose() {
    geometry_msgs::Pose registered_pose_= current_pose.pose;
    planAndMove(registered_pose_, std::string("registered pose"));
    visual_tools_.publishText(text_pose_, "Arrived at registered pose", rvt::WHITE, rvt::XLARGE);
  }


  private:
    geometry_msgs::PoseStamped current_pose;
    geometry_msgs::Pose base_pose_;

   
  };

 } //namespace move_to_target

 int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_group_pose_follower");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  move_to_target::TargMover registered(&node_handle, "manipulator", "world");
  ROS_INFO("Please start impedance mode and register your target. Click 'Next' to get back to base pose.");
  registered.waitForApproval();
  registered.moveToBasePose();
  ROS_INFO("Click 'Next' to move to the registered pose");
  registered.waitForApproval();
  registered.moveToRegisteredPose();

  /*
  ros::Rate rate(10);
  while(ros::ok()) {
    rate.sleep();
  }
  */
  ros::shutdown();
  return 0;
}
