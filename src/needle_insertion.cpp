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
--> Das in Imp.Mode?!

*/
#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <iimoveit/robot_interface.h>
#include <iiwa_registration/button_listener.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <moveit/transforms/transforms.h>
#include <tf/transform_datatypes.h>

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

//_________________________________________________________________
  void moveToBasePose() {
    planAndMove(base_pose_, std::string("base pose"));
  }

//_________________________________________________________________
  //waits for the SmartPad button to get pressed
  //and broadcasts the new transform from dummy_frame -> world
  //Tipp: have a look at the tf_tree before and after pressing the button.
  //In a Terminalwindow type:
  //  $ rosrun rqt_tf_tree rqt_tf_tree
  void buttonEventCallback(const std_msgs::String::ConstPtr& msg) {
    if(msg->data == "pose_get_pressed") {
        ROS_INFO("Received message: %s", msg->data.c_str());
        dummy_pose = getPose();
  	
        static tf2_ros::StaticTransformBroadcaster static_broadcaster;	
    	  
        static_transformStamped_dummy.header.stamp = ros::Time::now();
    	  static_transformStamped_dummy.header.frame_id = "world";
    	  static_transformStamped_dummy.child_frame_id = "dummy_frame";
    	  static_transformStamped_dummy.transform.translation.x = dummy_pose.pose.position.x;
    	  static_transformStamped_dummy.transform.translation.y = dummy_pose.pose.position.y;
    	  static_transformStamped_dummy.transform.translation.z = dummy_pose.pose.position.z;
    	  static_transformStamped_dummy.transform.rotation.x = dummy_pose.pose.orientation.x;
    	  static_transformStamped_dummy.transform.rotation.y = dummy_pose.pose.orientation.y;
    	  static_transformStamped_dummy.transform.rotation.z = dummy_pose.pose.orientation.z;
    	  static_transformStamped_dummy.transform.rotation.w = dummy_pose.pose.orientation.w;
    	  
    	  ROS_INFO("dummy_position = (%f, %f, %f), dummy_orientation (%f, %f, %f. %f)", dummy_pose.pose.position.x, dummy_pose.pose.position.y, dummy_pose.pose.position.z, dummy_pose.pose.orientation.x, dummy_pose.pose.orientation.y, dummy_pose.pose.orientation.z, dummy_pose.pose.orientation.w);
        
        //starting the broadcaster
      	
    insertion_pose_1.position.x = 0.0;
    insertion_pose_1.position.y = 0.0185;
    insertion_pose_1.position.z = 0.024;

    insertion_pose_2.position.x = 0.0;
    insertion_pose_2.position.y = 0.05;
    insertion_pose_2.position.z = 0.045;

    insertion_pose_3.position.x = 0.0;
    insertion_pose_3.position.y = 0.04;
    insertion_pose_3.position.z = 0.03;

    //manually calculated RPY angles (radians)for the three insertion axis (E1->target1, E2->target2, E3->target3)
    double r_1= 0.0, p_1 = 0.2, y_1 = 0.0; //E1->target1 => testvalue
    double r_2= 0, p_2 = 0, y_2 = 0; //E2->target2 => not yet set
    double r_3= 0, p_3 = 0, y_3 = 0; //E3->target3 => not yet set

    tf::Quaternion q_orig = tf::Quaternion::getIdentity();

    //compute quaternions for target frames 1...3
     
      q_rot_1 = tf::createQuaternionFromRPY(r_1, p_1, y_1);
      q_rot_2 = tf::createQuaternionFromRPY(r_2, p_2, y_2);
      q_rot_3 = tf::createQuaternionFromRPY(r_3, p_3, y_3);

      q_new_1 = q_rot_1*q_orig;  // Calculate the new orientation 
      q_new_2 = q_rot_2*q_orig;
      q_new_3 = q_rot_3*q_orig;

    //first frame
    
        
      static_transformStamped_needlePath_1.header.stamp = ros::Time::now();
      static_transformStamped_needlePath_1.header.frame_id = "dummy_frame";
      static_transformStamped_needlePath_1.child_frame_id = "target_frame_1";
      static_transformStamped_needlePath_1.transform.translation.x = insertion_pose_1.position.x;
      static_transformStamped_needlePath_1.transform.translation.y = insertion_pose_1.position.y;
      static_transformStamped_needlePath_1.transform.translation.z = insertion_pose_1.position.z;
      static_transformStamped_needlePath_1.transform.rotation.x = q_new_1.x();
      static_transformStamped_needlePath_1.transform.rotation.y = q_new_1.y();
      static_transformStamped_needlePath_1.transform.rotation.z = q_new_1.z();
      static_transformStamped_needlePath_1.transform.rotation.w = q_new_1.w();

     
 
    //second target
    
        
        static_transformStamped_needlePath_2.header.stamp = ros::Time::now();
        static_transformStamped_needlePath_2.header.frame_id = "dummy_frame";
        static_transformStamped_needlePath_2.child_frame_id = "target_frame_2";
        static_transformStamped_needlePath_2.transform.translation.x = insertion_pose_2.position.x;
        static_transformStamped_needlePath_2.transform.translation.y = insertion_pose_2.position.y;
        static_transformStamped_needlePath_2.transform.translation.z = insertion_pose_2.position.z;
        static_transformStamped_needlePath_2.transform.rotation.x = q_new_2.x();
        static_transformStamped_needlePath_2.transform.rotation.y = q_new_2.y();
        static_transformStamped_needlePath_2.transform.rotation.z = q_new_2.z();
        static_transformStamped_needlePath_2.transform.rotation.w = q_new_2.w();

        
    
    //third target
  
        
        static_transformStamped_needlePath_3.header.stamp = ros::Time::now();
        static_transformStamped_needlePath_3.header.frame_id = "dummy_frame";
        static_transformStamped_needlePath_3.child_frame_id = "target_frame_3";
        static_transformStamped_needlePath_3.transform.translation.x = insertion_pose_3.position.x;
        static_transformStamped_needlePath_3.transform.translation.y = insertion_pose_3.position.y;
        static_transformStamped_needlePath_3.transform.translation.z = insertion_pose_3.position.z;
        static_transformStamped_needlePath_3.transform.rotation.x = q_new_3.x();
        static_transformStamped_needlePath_3.transform.rotation.y = q_new_3.y();
        static_transformStamped_needlePath_3.transform.rotation.z = q_new_3.z();
        static_transformStamped_needlePath_3.transform.rotation.w = q_new_3.w();

        //static_broadcaster.sendTransform(static_transformStamped_dummy,static_transformStamped_needlePath_1,static_transformStamped_needlePath_2,static_transformStamped_needlePath_3);
        static_broadcaster.sendTransform(static_transformStamped_dummy);
        static_broadcaster.sendTransform(static_transformStamped_needlePath_1);
        static_broadcaster.sendTransform(static_transformStamped_needlePath_2);
        static_broadcaster.sendTransform(static_transformStamped_needlePath_3);
        ROS_INFO("Spinning until killed publishing %s to world", "base_pose");
    
    }
  }

  //_________________________________________________________________ 
  //Move to a point, where the needle tip will be on -0.02 m on the oriented 
  //insertionaxis for the first target
  //and insert the needle to the needletool 
  void moveToNeedletargetOne() {
  
    // We can print the name of the reference frame for this robot.
  ROS_INFO("working on moveToNeedletargetOne");

  // We can also print the name of the end-effector link for this group.
  ROS_INFO("End effector link: %s", move_group_.getEndEffectorLink().c_str());
  //change the referenceframe for the needleInsertionTarget pose to the registered dummy frame
  //and set end effector frame to needle tip frame "tool_link_nh"
      // We can print the name of the reference frame for this robot.
  ROS_INFO("Reference frame before setPoseReferenceFrame: %s", move_group_.getPoseReferenceFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  ROS_INFO("End effector link before setEndEffectorLink: %s", move_group_.getEndEffectorLink().c_str());
  
  needleInsertionTarget.position = insertion_pose_1.position;
  needleInsertionTarget.orientation.w = 1.0;
  needleInsertionTarget.position.x -= needleoffset;
  needleInsertionTarget.position.y = insertion_pose_1.position.y;
  needleInsertionTarget.position.z = insertion_pose_1.position.z;

  

 
  move_group_.setPoseReferenceFrame("target_frame_1");
  move_group_.setEndEffectorLink("tool_link_nh");
 ROS_INFO("PLANNING STARTED");
  planAndMove(needleInsertionTarget,"needle insertion pose");
ROS_INFO("PLANNING DONE");
  //move_group_.setPoseTarget(needleInsertionTarget, "tool_link_nh");

  
    // We can print the name of the reference frame for this robot.
  ROS_INFO("Reference frame after setPoseReferenceFrame: %s", move_group_.getPoseReferenceFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  ROS_INFO("End effector link after setEndEffectorLink: %s", move_group_.getEndEffectorLink().c_str());

  // move_group_.setPlanningTime(20.0);
  // move_group_.setGoalPositionTolerance(0.02);
  // move_group_.setGoalOrientationTolerance(0.03);
  // ROS_INFO("PLANNING STARTED");
  // moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  // ROS_INFO("PLANNING DONE");
  // bool success = (move_group_.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  // ROS_INFO("Visualizing plan() %s", success ? "" : "FAILED");

  // ROS_INFO("Visualizing plan as trajectory line");
  // visual_tools_.publishAxisLabeled(needleInsertionTarget, "target");
  // visual_tools_.publishText(text_pose_, "Ready to insert needle to tool", rvt::WHITE, rvt::XLARGE);
  // visual_tools_.publishTrajectoryLine(my_plan.trajectory_, joint_model_group_);
  // visual_tools_.trigger();
  // ROS_INFO("DONE VISUALIZING");
  // visual_tools_.prompt("next step");
  
  // move_group_.move();
  // updateRobotState();  
  }
  
  void cartesianMovementAlongTargetAxisOne() {
  
  //move forward along target_frame_1 axis
  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(needleInsertionTarget);

  move_group_.setPoseReferenceFrame("target_frame_1");
  move_group_.setEndEffectorLink("tool_link_nh");
  
  double forward = needleoffset + targetdepth_1;

  target_pose_1 = needleInsertionTarget;

  target_pose_1.position.x += forward;
  target_pose_1.orientation.w = 1.0;
  waypoints.push_back(target_pose_1);  //down along x-axis
  
  move_group_.setMaxVelocityScalingFactor(0.1);

 
  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double fraction = move_group_.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  ROS_INFO_NAMED("tutorial", "Visualizing plan (cartesian path) (%.2f%% acheived)", fraction * 100.0);

  //visualising
  visual_tools_.deleteAllMarkers();
  visual_tools_.publishText(text_pose_, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools_.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
  for (std::size_t i = 0; i < waypoints.size(); ++i)
    visual_tools_.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
  visual_tools_.trigger();
  visual_tools_.prompt("next step");
  
  move_group_.setPlanningTime(30.0);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  my_plan.trajectory_= trajectory;
  
  bool success = (move_group_.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
  
    move_group_.execute(my_plan);
    updateRobotState();

  visual_tools_.deleteAllMarkers();

  }


  private:
    //geometry_msgs::Pose registered_pose_ = dummy_pose.pose;
    ros::Subscriber us_subscriber_;
    double needleoffset = 0.2;
    double targetdepth_1 = 0.005, targetdepth_2 = 0.015, targetdepth_3 = 0.05;
    geometry_msgs::PoseStamped dummy_pose; //pose of the dummy
    geometry_msgs::Pose base_pose_;
    geometry_msgs::Pose needle_preinsertion_pose;
    geometry_msgs::Pose needleInsertionTarget;
    geometry_msgs::Pose insertion_pose_1, insertion_pose_2, insertion_pose_3;
    geometry_msgs::Pose target_pose_1, target_pose_2, target_pose_3;
    geometry_msgs::TransformStamped static_transformStamped_dummy;
    geometry_msgs::TransformStamped static_transformStamped_needlePath_1;
    geometry_msgs::TransformStamped static_transformStamped_needlePath_2;
    geometry_msgs::TransformStamped static_transformStamped_needlePath_3;
    tf::Quaternion q_rot_1, q_rot_2, q_rot_3, q_new_1, q_new_2, q_new_3;
};

} //namespace move_to_target

 int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_group_registration");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Rate rate(1000);
  
  move_to_target::TargMover registered(&node_handle, "manipulator", "world");

  registered.moveToBasePose(); //go to basePose to switch mode
 
  ROS_INFO("Please start impedance mode. Move the EEF to the target and register it by pressing 'get' on the SmartPad toolbar. Click 'Next' to move back to the basepose.");
  
  registered.waitForApproval();

  //ROS_INFO("computing insertion frames");
 // registered.computeInsertionFrames();

  ROS_INFO("moving back to base pose");
  registered.moveToBasePose();
  
  ROS_INFO("press 'next' to move to first targetpoint and get the right orientation");
  registered.waitForApproval();

  registered.moveToNeedletargetOne();

  ROS_INFO("press 'next' to insert needle");
  registered.waitForApproval();
  
  registered.cartesianMovementAlongTargetAxisOne();

  ROS_INFO("Inserting needle.");

	registered.waitForApproval();

	ros::shutdown();
	return 0;
}

