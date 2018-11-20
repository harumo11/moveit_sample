#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char* argv[])
{
	//setup for ros
	ros::init(argc, argv, "moveit_velocity");
	ros::NodeHandle hode_hadle;
	ros::AsyncSpinner spinner(1);
	spinner.start();

	//setup for moveit
	const std::string PLANNING_GROUP = "panda_arm";
	moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
	const robot_state::JointModelGroup* joint_model_group =
		move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

	//setup for rviz
	namespace rvt = rviz_visual_tools;
	moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
	visual_tools.deleteAllMarkers();
	visual_tools.loadRemoteControl();
	visual_tools.trigger();
	visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to move random pose");

	move_group.setRandomTarget();
	move_group.move();
	visual_tools.trigger();
	visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to move start pose");


	auto current_state = move_group.getCurrentState();
	geometry_msgs::Pose start_pose;
	start_pose.orientation.w = 1.0;
	start_pose.position.x =  0.55;
	start_pose.position.y = -0.25;
	start_pose.position.z = 0.8;
	move_group.setPoseTarget(start_pose);
	move_group.move();
	visual_tools.trigger();
	visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to move goal pose");

	std::vector<geometry_msgs::Pose> waypoints;
	waypoints.push_back(start_pose);
	auto speedup_pose = start_pose;
	speedup_pose.position.y = 0.0;
	waypoints.push_back(speedup_pose);
	auto goal_pose = speedup_pose;
	goal_pose.position.y = 0.25;
	waypoints.push_back(goal_pose);

	moveit_msgs::RobotTrajectory trajectory;
	const double jump_threshold = 0.0;
	const double eef_step = 0.01;
	double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
	robot_trajectory::RobotTrajectory rt(move_group.getCurrentState()->getRobotModel(), "panda_arm");
	rt.setRobotTrajectoryMsg(*move_group.getCurrentState(), trajectory);

	rt.setWayPointDurationFromPrevious(0, 10);

	trajectory_processing::IterativeParabolicTimeParameterization iptp;
	bool result = iptp.computeTimeStamps(rt);
	if (!result) {
		ROS_INFO("Cannot compute trajectory");
	}
	rt.getRobotTrajectoryMsg(trajectory);
	moveit::planning_interface::MoveGroupInterface::Plan plan;
	plan.trajectory_ = trajectory;
    ROS_INFO("Visualizing plan  (cartesian path) (%.2f%% acheived)",fraction * 100.0);   
	move_group.execute(plan);

	ROS_INFO("Compute Velocity trajectory");

	visual_tools.trigger();
	visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to finish");
	
	return 0;
}

