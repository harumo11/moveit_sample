cmake_minimum_required(VERSION 3.12)
project(my_moveit)

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

 add_executable(${PROJECT_NAME}_node src/my_moveit_node.cpp)
 add_executable(robot_state_node src/robot_state.cpp)
 add_executable(planning_scene_node src/planning_scene.cpp)
 add_executable(velocity_node src/velocity.cpp)

 set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

 add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 add_dependencies(robot_state_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 add_dependencies(planning_scene_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 add_dependencies(velocity_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

 target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
 )
 target_link_libraries(robot_state_node
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
 )

 target_link_libraries(planning_scene_node
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
 )

 target_link_libraries(velocity_node
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
 )
