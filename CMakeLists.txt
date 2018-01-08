cmake_minimum_required(VERSION 2.8.3)
project(iiwa_registration)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  iiwa_msgs
  )


catkin_package()


include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(button_listener src/button_listener.cpp)
target_link_libraries(button_listener ${catkin_LIBRARIES})
add_dependencies(button_listener ${catkin_EXPORTED_TARGETS})

