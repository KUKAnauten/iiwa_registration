#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_publisher");
  ros::NodeHandle n;

  ros::Rate r(10000);

  tf::TransformBroadcaster broadcaster;

  while(n.ok()){
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0.00964753,0.611417, -0.01133,0.791169), tf::Vector3(0.530517,-0.181726,1.16903)),
        ros::Time::now(),"/world", "/base_pose"));
    r.sleep();
  }
}
