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

//Author: Laura Bielenberg
namespace move_to_target{

  namespace rvt = rviz_visual_tools;


  class DummyDelta : public iimoveit::RobotInterface {
  public:
   DummyDelta(ros::NodeHandle* node_handle, const std::string& planning_group, const std::string& base_frame)
        : RobotInterface(node_handle, planning_group, base_frame) {   
        text_pose_.translation().z() = 2.3;
        
      base_pose_.header.frame_id ="world";  
      base_pose_.pose.position.x = 0.6;
      base_pose_.pose.position.y = -0.09;
      base_pose_.pose.position.z = 1.192;
      base_pose_.pose.orientation.x = 0.0;
      base_pose_.pose.orientation.y = 0.7071081;
      base_pose_.pose.orientation.z = 0.0;
      base_pose_.pose.orientation.w =  0.7071055;
     
  }

//_________________________________________________________________
  void moveToBasePose() {
    move_group_.setStartStateToCurrentState();
    planAndMove(base_pose_, std::string("base pose"));
  }

//_________________________________________________________________
  
  void moveAlongZAxisCartesian(double x_value, double y_value, double z_value){ //status: working!!

  //robot_state::RobotState start_state(*move_group_.getCurrentState());
  geometry_msgs::PoseStamped currentPose = getPose();

  
  geometry_msgs::PoseStamped start_pose = currentPose;

  move_group_.setPoseReferenceFrame("world");

  std::vector<geometry_msgs::Pose> waypoints;
  geometry_msgs::Pose target_pose = start_pose.pose;
 
  target_pose.position.x += x_value; 
  target_pose.position.y += y_value;
  target_pose.position.z += z_value;
  waypoints.push_back(target_pose);  
  
  ROS_INFO_STREAM("target pose is: " << target_pose);

  move_group_.setGoalOrientationTolerance(0.0);

  move_group_.setGoalPositionTolerance(0.0);

 // move_group_.setMaxVelocityScalingFactor(0.5);
 
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


  };

} //namespace move_to_target


int main(int argc, char **argv)
{
  ros::init(argc, argv, "dummy_delta");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Rate rate(10);
  
  int i = 0;
  // create a move_to_target object
  move_to_target::DummyDelta registered(&node_handle, "manipulator", "world");

  // create an iiwa_ros object for easy access to ROS topics and services of the iiwa robot
  iiwa_ros::iiwaRos iiwa_ros_object;

  iiwa_ros_object.init();

ROS_INFO("Moving to base_pose_ ");
    registered.moveToBasePose();
  
  for(i=1; i < 100; i++){

    ROS_INFO("Weiter mit Nadeleinstichschleife?");
    registered.waitForApproval();

    ROS_INFO("nadel zum %i mal einführen", i);
    registered.moveAlongZAxisCartesian(0.0, 0.0, -0.01);
    
    ROS_INFO("next' um nadel zum %i mal entnehmen", i);  
    
    registered.waitForApproval();
    
    registered.moveAlongZAxisCartesian( 0.0, 0.0, 0.01);
   }

  ros::shutdown();
  return 0;
}
