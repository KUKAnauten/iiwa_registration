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
  iiwa_ros
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

add_executable(tf_broadcaster src/tf_broadcaster.cpp)
target_link_libraries(tf_broadcaster ${catkin_LIBRARIES})

add_executable(needle_insertion src/needle_insertion.cpp)
target_link_libraries(needle_insertion ${catkin_LIBRARIES})

add_executable(ros_praktikum src/ros_praktikum.cpp)
target_link_libraries(ros_praktikum ${catkin_LIBRARIES})

#add_executable(tf_static_broadcaster src/tf_static_broadcaster.cpp)
#target_link_libraries(tf_static_broadcaster ${catkin_LIBRARIES})


add_executable(tf_listener src/tf_listener.cpp)
target_link_libraries(tf_listener ${catkin_LIBRARIES})

add_executable(test_planningFrame src/test_planningFrame.cpp)
target_link_libraries(test_planningFrame ${catkin_LIBRARIES})

add_dependencies(move_to_target ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
