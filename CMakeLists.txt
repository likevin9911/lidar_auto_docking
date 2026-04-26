cmake_minimum_required(VERSION 3.0.2)
project(lidar_auto_docking)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  sensor_msgs
  nav_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  angles
  actionlib
  actionlib_msgs
  message_generation
)

find_package(Eigen3 REQUIRED)

## Generate messages
add_message_files(
  FILES
  Initdock.msg
)

## Generate actions
add_action_files(
  FILES
  Dock.action
  Undock.action
)

## Generate added messages and services
generate_messages(
  DEPENDENCIES
  geometry_msgs
  actionlib_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES lidar_auto_docking
  CATKIN_DEPENDS
    roscpp
    rospy
    geometry_msgs
    sensor_msgs
    nav_msgs
    std_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    angles
    actionlib
    actionlib_msgs
    message_runtime
  DEPENDS
    EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

## dock_coordinates node
add_executable(dock_coordinates
  src/dock_coordinates.cpp
  src/perception.cpp
  src/laser_processor.cpp
  src/linear_pose_filter_2d.cpp
  src/icp_2d.cpp
  src/tf2listener.cpp
)
add_dependencies(dock_coordinates
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(dock_coordinates
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

## auto_dock node
add_executable(auto_dock
  src/autodock.cpp
  src/controller.cpp
  src/perception.cpp
  src/laser_processor.cpp
  src/linear_pose_filter_2d.cpp
  src/icp_2d.cpp
  src/tf2listener.cpp
)
add_dependencies(auto_dock
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(auto_dock
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

## Python scripts
catkin_install_python(PROGRAMS
  scripts/dock_robot.py
  scripts/undock_robot.py
  scripts/dock_saver.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

## Install launch, config, action, msg dirs
install(DIRECTORY
  launch
  config
  action
  msg
  initial_dock_pose
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
