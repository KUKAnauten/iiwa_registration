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
//#include <iiwa_registration/button_listener.h>


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

    needle_preinsertion_pose.position.x = insertion_point.x - 0.2; //offset to position the 19.5cm long needle
    needle_preinsertion_pose.position.y = insertion_point.y;
    needle_preinsertion_pose.position.z = insertion_point.z;
    needle_preinsertion_pose.orientation.x = box_pose.orientation.x;
    needle_preinsertion_pose.orientation.y = box_pose.orientation.y;
    needle_preinsertion_pose.orientation.z = box_pose.orientation.z;
    needle_preinsertion_pose.orientation.w = box_pose.orientation.w;

   
  }

  void buttonEventCallback(const std_msgs::String::ConstPtr& msg) {
    if(msg->data == "pose_get_pressed") {
      ROS_INFO("Der Schmartie funktioniert: %s", msg->data.c_str());
      dummy_pose = getPose();
      ROS_INFO("dummy Position = (%f, %f, %f)", dummy_pose.pose.position.x, dummy_pose.pose.position.y, dummy_pose.pose.position.z);
      visual_tools_.publishText(text_pose_, "Pose registered!", rvt::WHITE, rvt::XLARGE);
    }
  }

  void moveToBasePose() {
    planAndMove(base_pose_, std::string("base pose"));
    visual_tools_.publishText(text_pose_, "I'm at my base pose...press NEXT to get to the registered pose", rvt::WHITE, rvt::XLARGE);
  }
  
  //Move to a point above dummy, and get the right orientation (calculations in the making)
  void moveToNeedletarget() {

     planAndMove(needle_preinsertion_pose, std::string("registered pose"));
     visual_tools_.publishText(text_pose_, "Arrived at registered pose", rvt::WHITE, rvt::XLARGE);
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


  private:
    //geometry_msgs::Pose registered_pose_ = dummy_pose.pose;
    ros::Subscriber us_subscriber_;
    geometry_msgs::PoseStamped dummy_pose; //pose of the dummy
    geometry_msgs::Pose base_pose_;
    geometry_msgs::Pose needle_preinsertion_pose;
    geometry_msgs::Pose box_pose;
    geometry_msgs::Point insertion_point;

    void usCallback(const std_msgs::Float32::ConstPtr& msg) { //work in progress
    publishXGoal(0.1 * msg->data, 0.01);
  }
};

} //namespace move_to_target

 int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_group_pose_follower");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  move_to_target::TargMover registered(&node_handle, "manipulator", "world");

  registered.moveToBasePose(); //go to basePose to switch mode
  
  ROS_INFO("Please start impedance mode. Move the EEF to the target and register it by pressing 'get' on the SmartPad toolbar. Click 'Next' to move back to the basepose.");

  registered.waitForApproval();

  registered.moveToBasePose();

  ROS_INFO("Click 'Next' to spawn dummy.");

  registered.waitForApproval();

  registered.spawndummy();

  ROS_INFO("Click 'Next' to move to the registered pose");
  
  registered.waitForApproval();

  ROS_INFO("Click 'Next' to move to the registered pose");

  registered.moveToNeedletarget();

  ROS_INFO("Click 'Next' to insert needle");

  registered.waitForApproval();

  registered.insertNeedle();

  ROS_INFO("Click 'Next' to end");

  registered.waitForApproval();

  ros::shutdown();
  return 0;
}

