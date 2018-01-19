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
  iimoveit
  control_msgs
  iiwa_description
  iiwa_moveit
  moveit_msgs
  moveit_ros_planning_interface
  moveit_visual_tools
  rosmsg
  )


catkin_package()


include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(button_listener src/button_listener.cpp)
target_link_libraries(button_listener ${catkin_LIBRARIES})
add_dependencies(button_listener ${catkin_EXPORTED_TARGETS})

add_executable(move_to_target src/move_to_target.cpp)
target_link_libraries(move_to_target ${catkin_LIBRARIES})

add_executable(move_to_target_box src/move_to_target_box.cpp)
target_link_libraries(move_to_target_box ${catkin_LIBRARIES})

add_executable(boxtest src/boxtest.cpp)
target_link_libraries(boxtest ${catkin_LIBRARIES})

add_dependencies(move_to_target ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
