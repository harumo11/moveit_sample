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

	//move_groupクラスはプランニンググループの名前だけを用いてセットアップ出来ます
	moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

	//planning_sceneクラスを用いて接触可能な物体をバーチャル空間に出現させます
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

	//パフォーマンス改善のためにplanning groupはポインタを参照するときがあります
	const robot_state::JointModelGroup* joint_model_group = 
		move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

	//表示
	
	//
	//基本的な情報の取得
	//
	
	//ロボットに対する参照フレームの名前を表示
	ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group.getPlanningFrame().c_str());

	//ロボットのエンドエフェクタの名前を表示
	ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

	//ロボットの全グループのリストを表示
	ROS_INFO_NAMED("tutorial", "Available Planning Groups: ");
	//std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));

	
	//
	//ゴールへのプランニング
	//
	
	//現在の位置の表示
	std::cout << move_group.getCurrentPose() << std::endl;
	
	return 0;
}
