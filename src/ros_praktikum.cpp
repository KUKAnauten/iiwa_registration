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
#include <iiwa_ros/conversions.h>

//Author: Laura Bielenberg
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
      
      insertion_pose_1.position.x = 0.005;//0.005; 
      insertion_pose_1.position.y = 0.0185-0.0075;
      insertion_pose_1.position.z = 0.022-0.0075;

      // insertion_pose_2.position.x = 0.015;
      // insertion_pose_2.position.y = 0.05;
      // insertion_pose_2.position.z = 0.045;

      // insertion_pose_3.position.x = 0.05;
      // insertion_pose_3.position.y = 0.04;
      // insertion_pose_3.position.z = 0.03;

      //manually calculated RPY angles (radians) for the three insertion axis (E1->target1, E2->target2, E3->target3)
     
      double r_1= 0.0, p_1 = 0.0, y_1 = 0.0; //E1->target1 => testvalue
  
      tf::Quaternion q_orig = tf::Quaternion::getIdentity();

      //compute quaternions for target frames 1...3
      
        q_rot_1 = tf::createQuaternionFromRPY(r_1, p_1, y_1);
      
        q_new_1 = q_rot_1*q_orig;  // Calculate the new orientation 
       
        q_new_1.normalize();
       

        quaternionTFToMsg(q_new_1.normalize(), insertion_pose_1.orientation);

     
        ROS_INFO_STREAM("Quaternions 1 are: " << insertion_pose_1.orientation);
        
      // target_frame_1

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


      //starting the broadcaster
        //static_broadcaster.sendTransform(static_transformStamped_dummy,static_transformStamped_needlePath_1,static_transformStamped_needlePath_2,static_transformStamped_needlePath_3);
          static_broadcaster.sendTransform(static_transformStamped_dummy);
          static_broadcaster.sendTransform(static_transformStamped_needlePath_1);
    
          ROS_INFO("Broadcasting transforms from %s, to world and %s to dummy_frame", "dummy_frame","target_frame_1");
    
    }
  }
// All planning according to tool_link_ee !!!

  //position only: working! -> this is, what is needed, as "target_frame_1" already holds the needleorientation
  // needlelength is 0.195 m from ee to tip
  void moveToPositionRelativeTargetFrameOne(){
  
  //moving to point directly above target
  target_pose1.header.frame_id ="target_frame_1";
  target_pose1.pose.position.x = -0.22; // vom 0-Punkt des Targetkoordinatensystems aus 
  target_pose1.pose.position.y = 0.0;
  target_pose1.pose.position.z = 0.0;
  target_pose1.pose.orientation.w = 1.0;
  
  ROS_INFO("Reference Frame is: %s", move_group_.getPoseReferenceFrame().c_str());

  move_group_.setStartStateToCurrentState();
  planAndMove(target_pose1, std::string("target pose"));
  
  } 
  
  void setVelocity(double v){
   move_group_.setMaxVelocityScalingFactor(v);
  }

  void moveAlongXAxisCartesian(double x_value, double y_value, double z_value){ //status: working!!

  //robot_state::RobotState start_state(*move_group_.getCurrentState());
  geometry_msgs::PoseStamped currentPose = getPose();

  ROS_INFO_STREAM("startpose before transform is: " << currentPose);

  // is this extra transform really needed?
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

  //
  listener.transformPose("target_frame_1", currentPose, start_pose);

  ROS_INFO_STREAM("start pose after transform is: " << start_pose);

  move_group_.setPoseReferenceFrame("target_frame_1");

  std::vector<geometry_msgs::Pose> waypoints;
  geometry_msgs::Pose target_pose = start_pose.pose;

 // tf::Quaternion q_insert =  tf::Quaternion::getIdentity();
 // quaternionTFToMsg(q_insert.normalize(), target_pose.orientation);
  
 
  target_pose.position.x += x_value; 
  target_pose.position.y += y_value;
  target_pose.position.z += z_value;
  waypoints.push_back(target_pose);  
  
  ROS_INFO_STREAM("target pose is: " << target_pose);

  move_group_.setGoalOrientationTolerance(0.0);

  move_group_.setGoalPositionTolerance(0.0);

 
 
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
      geometry_msgs::PoseStamped target_pose1;

      geometry_msgs::Pose needle_preinsertion_pose;
      geometry_msgs::Pose needleInsertionTarget;
      geometry_msgs::Pose insertion_pose_1, insertion_pose_2, insertion_pose_3;
      geometry_msgs::Pose target_pose_1, target_pose_2, target_pose_3;

      geometry_msgs::TransformStamped static_transformStamped_dummy;
      geometry_msgs::TransformStamped static_transformStamped_needlePath_1;

      tf::Quaternion q_rot_1, q_new_1;
      tf::Quaternion q_new_tar1;
      tf::TransformListener listener;

  };

} //namespace move_to_target


int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_group_registration");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  //values for CartesianImpedanceMode servicecall
  // iiwa_msgs::CartesianQuantity stiffness_reg;
  // stiffness_reg.x = 20.0;
  // stiffness_reg.y = 20.0;
  // stiffness_reg.z = 20.0;
  // stiffness_reg.a = 10.0;
  // stiffness_reg.b = 10.0;
  // stiffness_reg.c = 10.0;
  // iiwa_msgs::CartesianQuantity damping_reg;
  // damping_reg.x = 0.8;
  // damping_reg.y = 0.8;
  // damping_reg.z = 0.8;
  // damping_reg.a = 0.8;
  // damping_reg.b = 0.8;
  // damping_reg.c = 0.8;

  ros::Rate rate(10);
  
  int i = 0;
  // create a move_to_target object
  move_to_target::TargMover registered(&node_handle, "manipulator", "world");

  // create an iiwa_ros object for easy access to ROS topics and services of the iiwa robot
  iiwa_ros::iiwaRos iiwa_ros_object;

  iiwa_ros_object.init();


  ROS_INFO("Moving to base_pose_ ");
  registered.moveToBasePose();

  ROS_INFO("Click 'next' to cange to CartesianImpedanceMode");
  registered.waitForApproval();

  iiwa_ros_object.getSmartServoService().setCartesianImpedanceMode(iiwa_ros::cartesianQuantityFromDouble(20,20,20,5,5,5), iiwa_ros::cartesianQuantityFromDouble(0.8));
  ROS_INFO("Changed Mode");
    
  ROS_INFO("Click 'next to change back to PositionControlMode '");
  registered.waitForApproval();

  iiwa_ros_object.getSmartServoService().setPositionControlMode();


  ROS_INFO("Planning to new pose above targetframe. Click 'next' to move.");
  registered.moveToPositionRelativeTargetFrameOne(); 

  for (int j = 1; j < 10; ++j)
  {
    double vel = 0.1*j;

    registered.setVelocity(vel);

    for(i=1; i < 10; i++){

      ROS_INFO("Weiter mit Nadeleinstichschleife?");
      registered.waitForApproval();

      ROS_INFO("nadel zum %i mal einführen", i);
      registered.moveAlongXAxisCartesian(0.034, 0.0, 0.0);
      
      ROS_INFO("next' um nadel zum %i mal entnehmen", i);  
      
      registered.waitForApproval();
      
      registered.moveAlongXAxisCartesian(-0.034, 0.0, 0.0);
     }

  }
  ros::shutdown();
    return 0;
}
