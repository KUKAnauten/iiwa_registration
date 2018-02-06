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
#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <iimoveit/robot_interface.h>
#include <iiwa_registration/button_listener.h>
#include <std_msgs/Float32.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/transforms/transforms.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>


#include <eigen_conversions/eigen_msg.h>

#include <Eigen/Geometry>

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

    //Position and Orientation of the Injectionaxis (seen as a new x-axis off the Entrypoint coordinatesystem) relative to the dummy Frame)
    insertion_point = dummy_pose.pose.position;
    insertion_point.x += 0.056;
    insertion_point.y += 0.0185;

    needle_preinsertion_pose.position.x = dummy_pose.pose.position.x; //offset to position the 19.5cm long needle
    needle_preinsertion_pose.position.y = dummy_pose.pose.position.y;
    needle_preinsertion_pose.position.z = dummy_pose.pose.position.z;

   
  }
  void calculateOrientation(){	 //set needle orientation (r,p,y need to be calculated beforehand)
	
	tf::Quaternion q_orig, q_rot, q_new;
	double r=0.0, p=0.0, y=0.1;  // Rotate the previous pose by 180* about X
	q_rot = tf::createQuaternionFromRPY(r, p, y);
	
	tf::Matrix3x3 mrot(q_rot);
	double rollROT, pitchROT, yawROT;
	mrot.getRPY(rollROT, pitchROT, yawROT);
	ROS_INFO("ROTATION: (%f,%f,%f)",rollROT,pitchROT,yawROT);
	q_new = q_rot*q_orig;  // Calculate the new orientation	
	

	quaternionMsgToTF(dummy_pose.pose.orientation , q_orig);  // Get the original orientation of 'commanded_pose'
	
	tf::Matrix3x3 m(q_orig);
	double rollOLD, pitchOLD, yawOLD;
	m.getRPY(rollOLD, pitchOLD, yawOLD);
	ROS_INFO("OLD POSITION: (%f,%f,%f)",rollOLD,pitchOLD,yawOLD);
	q_new = q_rot*q_orig;  // Calculate the new orientation
	
	tf::Matrix3x3 m2(q_new);
	double rollNEW, pitchNEW, yawNEW;
	m2.getRPY(rollNEW, pitchNEW, yawNEW);
	ROS_INFO("NEW POSITION: (%f,%f,%f)",rollNEW,pitchNEW,yawNEW);
	
	
	q_new.normalize();
	
	
	tf::Matrix3x3 m3(q_new);
	double rollNEWN, pitchNEWN, yawNEWN;
	m3.getRPY(rollNEWN, pitchNEWN, yawNEWN);
	ROS_INFO("NEW POSITION NORMALIZED: (%f,%f,%f)",rollNEWN,pitchNEWN,yawNEWN);
	
	
	
	quaternionTFToMsg(q_new, needle_preinsertion_pose.orientation);  // Stuff the new rotation back into the pose. This requires conversion into a msg type
	
	}

   void buttonEventCallback(const std_msgs::String::ConstPtr& msg) {
    if(msg->data == "pose_get_pressed") {
      ROS_INFO("Received message: %s", msg->data.c_str());
      dummy_pose = getPose();
	
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

  void moveToBasePose() {
    planAndMove(base_pose_, std::string("base pose"));
  }
  
  void changePlanningFrame(const std::string& pose_reference_frame){
  move_group_.setPoseReferenceFrame(pose_reference_frame);
  }
  
  //Move to a point above dummy, and get the right orientation (calculations in the making)
  void moveToNeedletarget() {
	geometry_msgs::Pose needleInsertionTarget;
  	needleInsertionTarget.orientation.w = 1.0;
  	
	needleInsertionTarget.position.x -= 0.1;
	
	move_group_.setPoseReferenceFrame("base_pose");
	move_group_.setPoseTarget(needleInsertionTarget, "tool_link_ee");
	
	moveit::planning_interface::MoveGroupInterface::Plan my_plan;

	bool success = (move_group_.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	

	ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

	ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
	visual_tools_.publishAxisLabeled(needleInsertionTarget, "pose1");
	visual_tools_.publishText(text_pose_, "Pose Goal", rvt::WHITE, rvt::XLARGE);
	visual_tools_.publishTrajectoryLine(my_plan.trajectory_, joint_model_group_);
	visual_tools_.trigger();
	visual_tools_.prompt("next step");
	
	move_group_.move();
     
  }


  void insertNeedle() {
    geometry_msgs::Pose target_pose = base_pose_;
    target_pose.position.x += 0.01; //tiefe Kugel 1: 0.005m + offset der nadel 0.005m
     planAndMove(target_pose, std::string("moved to target"));
     visual_tools_.publishText(text_pose_, "Arrived at target pose", rvt::WHITE, rvt::XLARGE);
  }

  void registerSubscriber() {
    us_subscriber_ = node_handle_->subscribe("/ultrasound", 1, &TargMover::usCallback, this);
  }

  //To move along the x-insertion axis
  void publishXGoal(double addition, double duration) { //work in progress
    geometry_msgs::Pose target_pose = base_pose_;
    target_pose.position.y += addition;
    publishPoseGoal(target_pose, duration);
  }


  // Only if no boxes are used in MoveIt
  // void moveToRegisteredPose() {
  //   planAndMove(registered_pose_, std::string("registered pose"));
  //   visual_tools_.publishText(text_pose_, "Arrived at registered pose", rvt::WHITE, rvt::XLARGE);
  // }


  
  void spawndummy() {

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = "world";
    collision_object.id = "tissue_dummy";

  //Silicondummy
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.07;
    primitive.dimensions[1] = 0.07;
    primitive.dimensions[2] = 0.0645;

    box_pose.position.x = dummy_pose.pose.position.x;
    box_pose.position.y = dummy_pose.pose.position.y;
    box_pose.position.z = dummy_pose.pose.position.z;
    box_pose.orientation.x = dummy_pose.pose.orientation.x;
    box_pose.orientation.y = dummy_pose.pose.orientation.y;
    box_pose.orientation.z = dummy_pose.pose.orientation.z;
    box_pose.orientation.w = dummy_pose.pose.orientation.w;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);
    std::cout << "collision_objects.size(): " << collision_objects.size() << std::endl;
    std::cout << collision_objects[0] << std::endl;
    planning_scene_interface.applyCollisionObjects(collision_objects);

    ros::Duration(2.0).sleep();

    std::vector<std::string> obj = planning_scene_interface.getKnownObjectNames();
    if (obj.size() > 0)
    {
      std::cout << std::endl << "-- KNOWN COLLISION OBJECTS --" << std::endl;
      for (int i = 0; i < obj.size(); ++i)
        std::cout << obj[i] << std::endl;
    }


    ROS_INFO_NAMED("tutorial", "Add an object into the world");
    planning_scene_interface.addCollisionObjects(collision_objects);

    ros::Duration(1.0).sleep();

  }
  
  void pathFromEigen(){
  	//Transformation of global coordinates to robot-specific coordinates
//	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
//	robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
//  	robot_state::RobotStatePtr kinematic_state = move_group_.getCurrentState();
//  	
//  	const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("manipulator");
//  	const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();
//  	std::vector<double> joint_values;
//	kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
//	for(std::size_t i = 0; i < joint_names.size(); ++i)
//	{
//	  ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
//	}
//  	
//  	const Eigen::Affine3d &end_effector_state = kinematic_state->getGlobalLinkTransform("tool_link_ee");
//  	geometry_msgs::Pose pose;
//	ROS_INFO_STREAM("TRANSFORMS: " << moveit::core::Transforms::getAllTransforms());
//	tf::TransformBroadcaster broadcaster;
//	broadcaster.sendTransform(
//	tf::StampedTransform(tf::Transform(tf::Quaternion(0.00964753,0.611417, -0.01133,0.791169), tf::Vector3(0.530517,-0.181726,1.16903)),
//        ros::Time::now(),"world", "base_pose"));
//	moveit::core::Transforms::setTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0.00964753,0.611417, -0.01133,0.791169), tf::Vector3(0.530517,-0.181726,1.16903)), ros::Time::now//(),"world", "base_pose"));
//	ROS_INFO_STREAM("TRANSFORMS2: " << moveit::core::Transforms::getAllTransforms());
	
//	    tf::Transform world_to_tool_tf;
//	    geometry_msgs::Pose world_to_tool;
//	 
//	    // lookup transform (this should be cached, since it’s probably static)
//	    tf_listener->lookupTransform(“world”, “tool_link_ee”, ros::Time(0.0f), world_to_tool_tf)
//	 
//	    // convert goal to TF data type, for easy math
//	    tf::poseMsgToTF(world_to_tool, world_to_tool_tf);
//	 
//	    // convert TF data type back to the type used in MoveIt Pose commands
//	    tf::poseTFToMsg(world_to_wrist_tf, world_to_wrist);
	 
	    // send command to moveIt
//	    move_group_.setPoseTarget(world_to_tool);
//	    move_group_.move();
  	geometry_msgs::Pose target;
  	target.orientation.w = 1.0;
  	
	target.position.x += 0.05;
	
	move_group_.setPoseReferenceFrame("base_pose");
	move_group_.setPoseTarget(target, "tool_link_ee");
	
	moveit::planning_interface::MoveGroupInterface::Plan my_plan;

	bool success = (move_group_.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	

	ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

	ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
	visual_tools_.publishAxisLabeled(target, "pose1");
	visual_tools_.publishText(text_pose_, "Pose Goal", rvt::WHITE, rvt::XLARGE);
	visual_tools_.publishTrajectoryLine(my_plan.trajectory_, joint_model_group_);
	visual_tools_.trigger();
	visual_tools_.prompt("next step");
	
	move_group_.move();
  	
  }
  void startTfListener(){
   tf::TransformListener listener;
  tf::StampedTransform transform;

  listener.waitForTransform("/base_pose", "/tool_link_ee", ros::Time(0), ros::Duration(10.0));
  listener.lookupTransform("/base_pose", "/tool_link_ee", ros::Time(0), transform);
	
  ROS_INFO_STREAM("frame_id:"<< transform.frame_id_);
  ROS_INFO_STREAM("child_frame:"<< transform.child_frame_id_);
  }
  void cartesianPath() {
  //cartesian path planner
 	std::vector<geometry_msgs::Pose> waypoints;
	waypoints.push_back(base_pose_);

	geometry_msgs::PoseStamped target_pose;
	target_pose.header.frame_id="tool_link_ee";
	target_pose.pose = base_pose_;


	target_pose.pose.position.x += 0.5;
	target_pose.pose.position.y += 0.6;
	target_pose.pose.position.z += 0.15;
	target_pose.pose.orientation.w = 1.0;
	waypoints.push_back(target_pose.pose);  // down and right
	
	move_group_.setMaxVelocityScalingFactor(0.1);
	
	moveit_msgs::RobotTrajectory trajectory;
	const double jump_threshold = 0.0;
	const double eef_step = 0.01;
	double fraction = move_group_.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
	ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (cartesian path) (%.2f%% acheived)", fraction * 100.0);

	//visualising
	visual_tools_.deleteAllMarkers();
	visual_tools_.publishText(text_pose_, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
	visual_tools_.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
	for (std::size_t i = 0; i < waypoints.size(); ++i)
	  visual_tools_.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
	visual_tools_.trigger();
	visual_tools_.prompt("next step");
	
	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	my_plan.trajectory_= trajectory;
	
	move_group_.execute(my_plan);
      	updateRobotState();
	}


  private:
    //geometry_msgs::Pose registered_pose_ = dummy_pose.pose;
    ros::Subscriber us_subscriber_;
    geometry_msgs::PoseStamped dummy_pose; //pose of the dummy
    geometry_msgs::Pose base_pose_;
    geometry_msgs::Pose needle_preinsertion_pose;
    geometry_msgs::Pose box_pose;
    geometry_msgs::Point insertion_point;
    geometry_msgs::TransformStamped static_transformStamped;
 //   tf::Transform transform;

    void usCallback(const std_msgs::Float32::ConstPtr& msg) { //work in progress
    publishXGoal(0.1 * msg->data, 0.01);
  }
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
  
//  registered.startTfListener();

  registered.moveToBasePose();
  
  registered.waitForApproval();
  
  registered.moveToNeedletarget();

  ROS_INFO("now computing Path from Eigen");
  
  registered.pathFromEigen();
//  registered.cartesianPath();

//	
// ROS_INFO("Please start impedance mode. Move the EEF to the target and register it by pressing 'get' on the SmartPad toolbar. Click 'Next' to move back to the basepose.");

// registered.waitForApproval();

//  registered.moveToBasePose();

//  ROS_INFO("Click 'Next' to spawn dummy for visualisation in Rviz.");

//  registered.waitForApproval();

    
    
//  registered.spawndummy();

//  ROS_INFO("Click 'Next' to move to the needleinsertion startpose");
////  
//  registered.waitForApproval();
//  
//  registered.changePlanningFrame("dummy");

//  registered.calculateOrientation();
 // registered.moveToNeedletarget();

//  ROS_INFO("Insert the needle and click 'Next' to move to the registered pose");


//  ROS_INFO("Click 'Next' to insert needle");

//  registered.waitForApproval();

//  registered.insertNeedle();

//  ROS_INFO("Click 'Next' to end");

	registered.waitForApproval();

	ros::shutdown();
	return 0;
}

