#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <iimoveit/robot_interface.h>
#include <std_msgs/Float32.h>
#include <tf/transform_listener.h>
#include <moveit/transforms/transforms.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Geometry>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

namespace move_to_target{

  namespace rvt = rviz_visual_tools;


  class TargMover : public iimoveit::RobotInterface {
  public:
   TargMover(ros::NodeHandle* node_handle, const std::string& planning_group, const std::string& base_frame)
        : RobotInterface(node_handle, planning_group, base_frame) {   
        text_pose_.translation().z() = 2.3;
        
      base_pose_.header.frame_id ="world";  
      base_pose_.pose.position.x = 0.530517;
      base_pose_.pose.position.y = -0.181726;
      base_pose_.pose.position.z = 1.16903;
      base_pose_.pose.orientation.x = 0.00964753;
      base_pose_.pose.orientation.y = 0.611417;
      base_pose_.pose.orientation.z = -0.01133;
      base_pose_.pose.orientation.w = 0.791169;
     
  }

//_________________________________________________________________
  void moveToBasePose() {
    move_group_.setStartStateToCurrentState();
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

       // q_new_1.normalize();
        q_new_1.normalize();
        q_new_1.normalize();


        quaternionTFToMsg(q_new_1.normalize(), insertion_pose_1.orientation);
        quaternionTFToMsg(q_new_2, insertion_pose_2.orientation);
        quaternionTFToMsg(q_new_3, insertion_pose_3.orientation);
      //first frame
        ROS_INFO_STREAM("Quaternions 1 are: " << insertion_pose_1.orientation);
          
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


      //starting the broadcaster
        //static_broadcaster.sendTransform(static_transformStamped_dummy,static_transformStamped_needlePath_1,static_transformStamped_needlePath_2,static_transformStamped_needlePath_3);
          static_broadcaster.sendTransform(static_transformStamped_dummy);
          static_broadcaster.sendTransform(static_transformStamped_needlePath_1);
          static_broadcaster.sendTransform(static_transformStamped_needlePath_2);
          static_broadcaster.sendTransform(static_transformStamped_needlePath_3);
          ROS_INFO("Spinning until killed publishing %s to world", "base_pose");
    
    }
  }
// All planning according to tool_link_ee !!!

  //position only: working! -> this is, what is needed, as "target_frame_1" already holds the needleorientation
  void moveToPositionRelativeTargetFrameOne(){

  
  
  target_pose1.header.frame_id ="target_frame_1";
  target_pose1.pose.position.x = -0.2;
  target_pose1.pose.position.y = 0.0075;
  target_pose1.pose.position.z = 0.0;//offset from registration part of the tool to the point that holds the needle
  target_pose1.pose.orientation.w = 1.0;
  
  ROS_INFO("Reference Frame is: %s", move_group_.getPlanningFrame().c_str());

  move_group_.setPlanningTime(100);

  move_group_.setStartStateToCurrentState();
  planAndMove(target_pose1, std::string("target pose"));
  
  } 
  
  //orientation only: working!
  void moveToOrientationRelativeTargetFrameOne(){

  double r1 = 0.1, p1 = 0.0, y1=0.0; //desired RPY
  tf::Quaternion q_new_tar1, q_rot_tar1 = tf::createQuaternionFromRPY(r1, p1, y1);

  q_new_tar1=q_rot_tar1*q_new_1;


  geometry_msgs::PoseStamped target_pose1;
  target_pose1.header.frame_id ="target_frame_1";
  target_pose1.pose.position.x = 0.0;
  target_pose1.pose.position.y = 0.0;
  target_pose1.pose.position.z = 0.0;

  quaternionTFToMsg(q_new_tar1, target_pose1.pose.orientation);
 
  ROS_INFO("Reference Frame is: %s", move_group_.getPlanningFrame().c_str());

  move_group_.setPlanningTime(100);

  move_group_.setStartStateToCurrentState();
  planAndMove(target_pose1, std::string("target pose"));
  
  } 

  //both position and orientation: working! (unstable)
  void moveToPoseRelativeTargetOne(){

  double r1 = 0.1, p1 = 0.0, y1=0.0; //desired RPY
  tf::Quaternion q_rot_tar1 = tf::createQuaternionFromRPY(r1, p1, y1);

  q_new_tar1=q_rot_tar1*q_new_1;

  geometry_msgs::PoseStamped target_pose1;
  target_pose1.header.frame_id ="target_frame_1";
  target_pose1.pose.position.x = -0.2;
  target_pose1.pose.position.y = 0.0;
  target_pose1.pose.position.z = 0.0;
  
  quaternionTFToMsg(q_new_tar1, target_pose1.pose.orientation);
  
  ROS_INFO("Reference Frame is: %s", move_group_.getPlanningFrame().c_str());

  move_group_.setPlanningTime(100);

  move_group_.setStartStateToCurrentState();
  planAndMove(target_pose1, std::string("target pose"));
  
  } 

  void moveAlongXAxisCartesian(){ //status: working!!

  //robot_state::RobotState start_state(*move_group_.getCurrentState());
  geometry_msgs::PoseStamped currentPose = getPose();

  ROS_INFO_STREAM("start pose before transform is: " << currentPose);

  
      tf::StampedTransform transform;
      try{
        listener.waitForTransform("target_frame_1", "world", ros::Time(0), ros::Duration(10.0) );
        listener.lookupTransform("target_frame_1", "world",ros::Time(0), transform);
      }
      catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
      }

  
  geometry_msgs::PoseStamped start_pose;

  listener.transformPose("target_frame_1", currentPose, start_pose);

  ROS_INFO_STREAM("start pose after transform is: " << start_pose);

  move_group_.setPoseReferenceFrame("target_frame_1");

  std::vector<geometry_msgs::Pose> waypoints;
  geometry_msgs::Pose target_pose = start_pose.pose;

  tf::Quaternion q_insert =  tf::Quaternion::getIdentity();
  quaternionTFToMsg(q_insert.normalize(), target_pose.orientation);

  ROS_INFO_STREAM("target pose is: " << target_pose);
 
  target_pose.position.x += 0.2; // test
  waypoints.push_back(target_pose);  // down along xaxis


  move_group_.setGoalOrientationTolerance(0.2);

  move_group_.setMaxVelocityScalingFactor(0.1);
 
  moveit_msgs::RobotTrajectory trajectory;
  move_group_.setPlanningTime(10);

  double fraction;
  for(int attempts = 0; attempts < 10; attempts++){
    fraction = move_group_.computeCartesianPath(waypoints,
                                               0.01,  // eef_step
                                               10.0,   // jump_threshold 10 -> TODO: see if less works
                                               trajectory, false);
    ROS_INFO("attempts count:%d",attempts);
    if(fraction >= 1){
      break;
    }
  }

  // The trajectory needs to be modified so it will include velocities as well. This is done here.
  // First to create a RobotTrajectory object
  robot_trajectory::RobotTrajectory rt(move_group_.getCurrentState()->getRobotModel(), "manipulator");
 
  // Second get a RobotTrajectory from trajectory
  rt.setRobotTrajectoryMsg(*move_group_.getCurrentState(), trajectory);
 
  // Thrid create a IterativeParabolicTimeParameterization object
  trajectory_processing::IterativeParabolicTimeParameterization iptp;

  // Fourth compute computeTimeStamps
  bool success = iptp.computeTimeStamps(rt);
  ROS_INFO("Computed time stamp %s",success?"SUCCEDED":"FAILED");

  // Get RobotTrajectory_msg from RobotTrajectory
  rt.getRobotTrajectoryMsg(trajectory);

  // Finally plan and execute the trajectory
  move_group_.setStartStateToCurrentState(); //?
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  my_plan.trajectory_= trajectory;
  ROS_INFO("Pose Reference Frame: %s",move_group_.getPoseReferenceFrame().c_str());
  ROS_INFO("Reference Frame: %s",move_group_.getPlanningFrame().c_str());
  ROS_INFO_NAMED("tutorial", "Visualizing plan (cartesian path) (%.2f%% acheived)", fraction * 100.0);
  visual_tools_.publishTrajectoryLine(my_plan.trajectory_, joint_model_group_);
  visual_tools_.trigger(); 
<<<<<<< HEAD
  

  sleep(5.0); //wait 5sec -> TODO: swap for waitforapproval

  move_group_.execute(my_plan);
=======
>>>>>>> 67c02d9e6b2196884d0c90f117ba8b58a1712926

    if(success){
      visual_tools_.prompt("Continue with moving?");
      visual_tools_.publishText(text_pose_, "Moving to pose", rvt::WHITE, rvt::XLARGE);
      visual_tools_.trigger();
      move_group_.execute(my_plan);
      updateRobotState();
    }
}
  



<<<<<<< HEAD
  
=======
  void moveToBaseWithPathConstraints(){

  moveit_msgs::OrientationConstraint ocm;
  ocm.link_name = "tool_link_ee";
  ocm.header.frame_id = "world";

  ocm.absolute_x_axis_tolerance = 1.0;
  ocm.absolute_y_axis_tolerance = 1.0;
  ocm.absolute_z_axis_tolerance = 1.0;
  ocm.weight = 1.0;

  moveit_msgs::Constraints test_constraints;
  test_constraints.orientation_constraints.push_back(ocm);
  move_group_.setPathConstraints(test_constraints);

  moveToBasePose();

  move_group_.clearPathConstraints();
  

}
>>>>>>> 67c02d9e6b2196884d0c90f117ba8b58a1712926
     private:
      //geometry_msgs::Pose registered_pose_ = dummy_pose.pose;
      ros::Subscriber us_subscriber_;
      geometry_msgs::PoseStamped dummy_pose; //pose of the dummy
      geometry_msgs::PoseStamped base_pose_;
      geometry_msgs::Pose needle_preinsertion_pose;
      geometry_msgs::Pose needleInsertionTarget;
      geometry_msgs::Pose insertion_pose_1, insertion_pose_2, insertion_pose_3;
      geometry_msgs::Pose target_pose_1, target_pose_2, target_pose_3;
      geometry_msgs::TransformStamped static_transformStamped_dummy;
      geometry_msgs::TransformStamped static_transformStamped_needlePath_1;
      geometry_msgs::TransformStamped static_transformStamped_needlePath_2;
      geometry_msgs::TransformStamped static_transformStamped_needlePath_3;
      tf::Quaternion q_rot_1, q_rot_2, q_rot_3, q_new_1, q_new_2, q_new_3;
      tf::Quaternion q_new_tar1;
      tf::TransformListener listener;
      geometry_msgs::PoseStamped target_pose1;
  };

} //namespace move_to_target


int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_group_registration");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Rate rate(10);
  
  move_to_target::TargMover registered(&node_handle, "manipulator", "world");

  ROS_INFO("Moving to base_pose_");
  registered.moveToBaseWithPathConstraints();

   ROS_INFO("Waiting for approval");
  registered.waitForApproval();

  ROS_INFO("Moving to new pose above targetframe");
  registered.moveToPositionRelativeTargetFrameOne();

  ROS_INFO("Waiting for approval");
  registered.waitForApproval();
  
  ROS_INFO("start planing for needleinsertion");
  registered.moveAlongXAxisCartesian();
  
  ROS_INFO("finished planning");  
  
  registered.waitForApproval();
  
  registered.moveToBasePose();
   
  ros::shutdown();
  return 0;
}
