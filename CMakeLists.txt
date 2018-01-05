cmake_minimum_required(VERSION 2.8.3)
project(iiwa_registration)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
	)


catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

