#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>
#include <iimoveit/robot_interface.h>
#include <std_msgs/Float32.h>
#include <tf/transform_listener.h>
#include <moveit/transforms/transforms.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Geometry>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <iiwa_ros.h>
#include <iiwa_msgs/CartesianQuantity.h>


namespace move_to_target{

  namespace rvt = rviz_visual_tools;


  class TargMover : public iimoveit::RobotInterface {
  public:
   TargMover(ros::NodeHandle* node_handle, const std::string& planning_group, const std::string& base_frame)
        : RobotInterface(node_handle, planning_group, base_frame) {   
        text_pose_.translation().z() = 2.3;
        
      base_pose_.header.frame_id ="world";  
      base_pose_.pose.position.x = 0.536;
      base_pose_.pose.position.y = -0.011;
      base_pose_.pose.position.z = 1.175;
      base_pose_.pose.orientation.x = 0.806;
      base_pose_.pose.orientation.y = 0.025;
      base_pose_.pose.orientation.z = -0.590;
      base_pose_.pose.orientation.w = 0.036;
     
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
  //In a new terminalwindow type:
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
        
      /*The new tf frames Position is defined by the targets positions relative to the dummy coordinatesystem.
      	This means, the targets set the origin of the new frames target_frame_xyz(0,0,0)*/	
      
      insertion_pose_1.position.x = 0.005;
      insertion_pose_1.position.y = 0.0185+0.075;
      insertion_pose_1.position.z = 0.024+0.075;

      insertion_pose_2.position.x = 0.015;
      insertion_pose_2.position.y = 0.05;
      insertion_pose_2.position.z = 0.045;

      insertion_pose_3.position.x = 0.05;
      insertion_pose_3.position.y = 0.04;
      insertion_pose_3.position.z = 0.03;

      //manually calculated RPY angles (radians) for the three insertion axis (E1->target1, E2->target2, E3->target3)
     
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

        q_new_1.normalize();
        q_new_2.normalize();
        q_new_3.normalize();


        quaternionTFToMsg(q_new_1.normalize(), insertion_pose_1.orientation);
         // quaternionTFToMsg(q_new_2, insertion_pose_2.orientation);
         // quaternionTFToMsg(q_new_3, insertion_pose_3.orientation);

     
        ROS_INFO_STREAM("Quaternions 1 are: " << insertion_pose_1.orientation);
        
      // target_frame_1

        static_transformStamped_needlePath_1.header.stamp = ros::Time::now();
        static_transformStamped_needlePath_1.header.frame_id = "dummy_frame";
        static_transformStamped_needlePath_1.child_frame_id = "target_frame_1";
        static_transformStamped_needlePath_2.transform.translation.x = insertion_pose_1.position.x;
        static_transformStamped_needlePath_2.transform.translation.y = insertion_pose_1.position.y;
        static_transformStamped_needlePath_2.transform.translation.z = insertion_pose_1.position.z;
        static_transformStamped_needlePath_2.transform.rotation.x = q_new_1.x();
        static_transformStamped_needlePath_1.transform.rotation.y = q_new_1.y();
        static_transformStamped_needlePath_1.transform.rotation.z = q_new_1.z();
        static_transformStamped_needlePath_1.transform.rotation.w = q_new_1.w();
       
   
      //target_frame_2
        
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

              
      //target_frame_3
    
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
          ROS_INFO("Broadcasting transforms from %s, to world and %s, %s, %s to dummy_frame", "dummy_frame","target_frame_1","target_frame_2","target_frame_3");
    
    }
  }
// All planning according to tool_link_ee !!!

  //position only: working! -> this is, what is needed, as "target_frame_1" already holds the needleorientation
  void moveToPositionRelativeTargetFrameOne(){
  
  target_pose1.header.frame_id ="target_frame_1";
  target_pose1.pose.position.x = -0.22;
  target_pose1.pose.position.y = 0.025;
  target_pose1.pose.position.z = 0.025;//offset from registration part of the tool to the point that holds the needle
  target_pose1.pose.orientation.w = 1.0;
  
  ROS_INFO("Reference Frame is: %s", move_group_.getPoseReferenceFrame().c_str());

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
  target_pose1.pose.position.x = -0.195;
  target_pose1.pose.position.y = 0.0;
  target_pose1.pose.position.z = 0.0;
  
  quaternionTFToMsg(q_new_tar1, target_pose1.pose.orientation);
  
  ROS_INFO("Reference Frame is: %s", move_group_.getPlanningFrame().c_str());

  move_group_.setPlanningTime(100);

  move_group_.setStartStateToCurrentState();
  planAndMove(target_pose1, std::string("target pose"));
  
  } 

  void moveAlongXAxisCartesian(double x_value, double y_value, double z_value){ //status: working!!

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

 
  target_pose.position.x += x_value; 
  target_pose.position.x += y_value;
  target_pose.position.x += z_value;
  waypoints.push_back(target_pose);  
  
  ROS_INFO_STREAM("target pose is: " << target_pose);

  move_group_.setGoalOrientationTolerance(0.0);

  move_group_.setGoalPositionTolerance(0.0);

  move_group_.setMaxVelocityScalingFactor(0.5);
 
  moveit_msgs::RobotTrajectory trajectory;
  move_group_.setPlanningTime(50);

  double fraction = move_group_.computeCartesianPath(waypoints,
                                               0.01,  // eef_step
                                               0.0,   
                                               trajectory, false);

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

    if(success){
      visual_tools_.prompt("Continue with moving?");
      visual_tools_.publishText(text_pose_, "Moving to pose", rvt::WHITE, rvt::XLARGE);
      visual_tools_.trigger();
      move_group_.execute(my_plan);
      updateRobotState();
    }
}
  

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

  //values for CartesianImpedanceMode servicecall
  iiwa_msgs::CartesianQuantity stiffness_reg;
  stiffness_reg.x = 20.0;
  stiffness_reg.y = 20.0;
  stiffness_reg.z = 20.0;
  stiffness_reg.a = 20.0;
  stiffness_reg.b = 20.0;
  stiffness_reg.c = 20.0;
  iiwa_msgs::CartesianQuantity damping_reg;
  damping_reg.x = 0.8;
  damping_reg.y = 0.8;
  damping_reg.z = 0.8;
  damping_reg.a = 0.8;
  damping_reg.b = 0.8;
  damping_reg.c = 0.8;

  iiwa_msgs::CartesianQuantity stiffness_inj;
  stiffness_inj.x = 40.0;
  stiffness_inj.y = 100.0;
  stiffness_inj.z = 100.0;
  stiffness_inj.a = 100.0;
  stiffness_inj.b = 100.0;
  stiffness_inj.c = 100.0;
  iiwa_msgs::CartesianQuantity damping_inj;
  damping_inj.x = 0.9;
  damping_inj.y = 0.9;
  damping_inj.z = 0.9;
  damping_inj.a = 0.9;
  damping_inj.b = 0.9;
  damping_inj.c = 0.9;

  ros::Rate rate(10);
  
  // create a move_to_target object
  move_to_target::TargMover registered(&node_handle, "manipulator", "world");

  // create an iiwa_ros object for easy access to ROS topics and services of the iiwa robot
  iiwa_ros::iiwaRos iiwa_ros_object;

  iiwa_ros_object.init();


  ROS_INFO("Moving to base_pose_ ");
  registered.moveToBasePose();

  ROS_INFO("Click 'next' to cange to CartesianImpedanceMode");
  registered.waitForApproval();

  iiwa_ros_object.getSmartServoService().setCartesianImpedanceMode(stiffness_reg, damping_reg, 0.0, 0.7);
  ROS_INFO("Changed Mode");
  	
  ROS_INFO("Click 'next to change back to PositionControlMode '");
  registered.waitForApproval();

  iiwa_ros_object.getSmartServoService().setPositionControlMode();

  ROS_INFO("Move above targetframe?");
  registered.waitForApproval();

  ROS_INFO("Moving to new pose above targetframe");
  registered.moveToPositionRelativeTargetFrameOne(); 

  ROS_INFO("Waiting for approval");
  registered.waitForApproval();
  
  ROS_INFO("start planning for needleinsertion");
  registered.moveAlongXAxisCartesian(0.06, 0.0, 0.0);
  
  ROS_INFO("finished planning");  
  
  registered.waitForApproval();
  
  registered.moveAlongXAxisCartesian(-0.1, 0.0, 0.0);
   
  iiwa_ros_object.getSmartServoService().setPositionControlMode();

  ros::shutdown();
  return 0;
}
