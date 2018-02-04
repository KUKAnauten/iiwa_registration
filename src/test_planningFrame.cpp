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

/*TODO*/
/*
- Ausrichten (Quaternionen) -> als reconfig. Parameter? ->needle_chanal_pose über topic?
- Block weg
- Ziel in Block
- bewegung zum ziel im Block (constraint in Z)
--> Das in Imp.Mode?!

*/

#include <iimoveit/robot_interface.h>
#include <std_msgs/Float32.h>
#include <eigen_conversions/eigen_msg.h>


namespace move_to_target{

  namespace rvt = rviz_visual_tools;


class TargMover : public iimoveit::RobotInterface {
public:
 TargMover(ros::NodeHandle* node_handle, const std::string& planning_group, const std::string& base_frame)
      : RobotInterface(node_handle, planning_group, base_frame) {
    base_pose_.position.x = 0.530517;
    base_pose_.position.y = -0.181726;
    base_pose_.position.z = 1.16903;
    base_pose_.orientation.x = 0.00964753;
    base_pose_.orientation.y = 0.611417;
    base_pose_.orientation.z = -0.01133;
    base_pose_.orientation.w = 0.791169;
   
    
    }
    
  	
	
  void moveToBasePose() {
    planAndMove(base_pose_, std::string("base pose"));
  }

  void registerSubscriber() {
    sinus_subscriber_ = node_handle_->subscribe("/sinus", 1, &TargMover::sinusCallback, this);
  }
  

  void publishYGoal(double addition, double duration) {
    	geometry_msgs::Pose target_pose = pose;
    	target_pose.position.z += addition;
    	publishPoseGoal(target_pose, duration);
  }
  
  void changePlanningFrame(const std::string& pose_reference_frame){
  move_group_.setPoseReferenceFrame(pose_reference_frame);
  }
  
  void testFunction(){
  const Eigen::Affine3d &end_effector_state = move_group_->getGlobalLinkTransform("tool_link_ee");
geometry_msgs::Pose pose;
tf::poseEigenToMsg(end_effector_state, pose);
pose.position.y -= 0.05;
move_group_.setPoseTarget(pose);
  }

private:
  ros::Subscriber sinus_subscriber_;
  geometry_msgs::Pose base_pose_;
  geometry_msgs::Pose pose;

  void sinusCallback(const std_msgs::Float32::ConstPtr& msg) {
    publishYGoal(0.1 * msg->data, 0.01);
  }
};
} // namespace sinus_follower

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_group_sintest");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  move_to_target::TargMover sinus_follower(&node_handle, "manipulator", "world");
  sinus_follower.moveToBasePose();
  sinus_follower.waitForApproval();
  
//  moveit_msgs::OrientationConstraint ocm;
//  ocm.link_name = "tool_link_ee";
//  ocm.header.frame_id = "base_link"; //to dummy_link
//  ocm.orientation.w = 1.0;
//  ocm.absolute_x_axis_tolerance = 0.1;
//  ocm.absolute_y_axis_tolerance = 0.1;
//  ocm.absolute_z_axis_tolerance = 0.1;
//  ocm.weight = 1.0
//  
//  moveit_msgs::Constraints test_constraints;
//  test_constraints.orientation_constraints.push_back(ocm);
//  move_group.setPathConstraints(test_constraints);

//  sinus.testFunction();

  sinus_follower.registerSubscriber();
  ROS_INFO_NAMED("iiwa_test", "Subscribed to sinus!");


  ros::Rate rate(10);
  while(ros::ok()) {
    rate.sleep();
  }
  ros::shutdown();
  return 0;
}

