#include <iostream>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char* argv[])
{
	//Initialize
	ros::init(argc, argv, "my_moveit_node");
	ros::NodeHandle node_handle;
	ros::AsyncSpinner spinner(1);
	spinner.start();

	//SetUP
	//panda_arm : Planning Group (rviz/MotionPlaning/Planning Request/Planning Group)
	static const std::string PLANNING_GROUP = "panda_arm";

	moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

	const robot_state::JointModelGroup* joint_model_group = 
		move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

	
	return 0;
}
