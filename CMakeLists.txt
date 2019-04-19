cmake_minimum_required(VERSION 3.10)
project(moveit_start)

set(CMAKE_EXPORT_COMPILE_COMMANDS on)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  control_msgs
  geometry_msgs
  moveit_commander
  moveit_core
  moveit_msgs
  moveit_ros_planning_interface
  moveit_visual_tools
  roscpp
  shape_msgs
  tf2
  trajectory_msgs
)

## System dependencies are found with CMake's conventions
 find_package(Boost REQUIRED 
	 COMPONENTS
	 system
	 date_time
	 filesystem
	 thread
)


catkin_package(
  CATKIN_DEPENDS 
  actionlib 
  control_msgs 
  geometry_msgs 
  moveit_commander 
  moveit_core 
  moveit_msgs 
  moveit_ros_planning_interface 
  moveit_visual_tools 
  roscpp 
  shape_msgs 
  tf2 
  trajectory_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

