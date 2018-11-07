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
	std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));

	
	//
	//ゴールへのプランニング
	//
	
	//現在の位置の表示
	std::cout << move_group.getCurrentPose() << std::endl;

	//目標座標
	geometry_msgs::Pose target_pose1;
	target_pose1.orientation.w = 1.0;
	target_pose1.position.x = 0.28;
	target_pose1.position.y = -0.2;
	target_pose1.position.z = 0.5;
	move_group.setPoseTarget(target_pose1);

	//プランナーでプラン作成・表示
	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	if (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
		ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) success");
	}
	else {
		ROS_WARN_NAMED("tutorial", "Visualizing plan 1 (pose goal) failed");
	}
	
	move_group.move();

	//
	//Joint空間でのプランニング
	//
	
	//現在のrobot stateへのポインタを得る．robot stateには現時点の全部(関節角?)の位置・速度・加速度が含まれている
	moveit::core::RobotStatePtr current_state = move_group.getCurrentState();

	//次に，現在のrobot stateから現在の関節角度の値を得る
	//joint_model_groupは同時変換行列の集合なので，これを用いてlocal角度をworld角度に変換しているのかも
	std::vector<double> joint_group_positions;
	current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

	//今回は１つの軸だけを動かします．そのためのゴールを作成します
	joint_group_positions.at(0) = -1; //radians
	move_group.setJointValueTarget(joint_group_positions);

	//動作プランを作成します
	if (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
		ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) success");
	}
	else {
		ROS_WARN_NAMED("tutorial", "Visualizing plan 2 (joint space goal) failed");
	}

	move_group.move();


	//
	//パス制約とともにプランニング
	//
	
	//パス制約はロボットのリンクに対して簡単に指定できます．
	//では，パス制約とゴールポーズを我々のグループに対して設定しましょう
	//まず，パス制約です．
	moveit_msgs::OrientationConstraint ocm;
	ocm.link_name = "panda_link7";
	ocm.header.frame_id = "panda_link0";
	ocm.orientation.w = 1.0;
	ocm.absolute_x_axis_tolerance = 0.1;
	ocm.absolute_y_axis_tolerance = 0.1;
	ocm.absolute_z_axis_tolerance = 0.1;
	ocm.weight = 1.0;

	//では，それをパス制約としてグループにセットしましょう
	moveit_msgs::Constraints test_constrains;
	test_constrains.orientation_constraints.push_back(ocm);
	move_group.setPathConstraints(test_constrains);

	//我々がすでにプランニングした古いゴールを再利用します
	//注意：これは現在の状態がすでにパス制約を満足している場合にのみうまく動きます．
	//なので，われわれはスタート状態を新しいポーズにセットしないといけません．
	robot_state::RobotState start_state(*move_group.getCurrentState());
	geometry_msgs::Pose start_pose2;
	start_pose2.orientation.w = 1.0;
	start_pose2.position.x = 0.55;
	start_pose2.position.y = -0.05;
	start_pose2.position.z = 0.8;
	//同時変換行列の集合と手先位置によりIKをといて各関節の目標値をセットする
	start_state.setFromIK(joint_model_group, start_pose2);
	//それをスタートポーズとする
	move_group.setStartState(start_state);

	//ゴールポジションを作る
	move_group.setPoseTarget(target_pose1);

	//パス制約とともにプランニングする場合は計算が遅くなるかもしれません
	//そこでプランニング時間を増やしましょう．
	//デフォルトでは５秒ですが，プランニングがうまく成功するようにそれ以上にするといいでしょう
	move_group.setPlanningTime(10.0);

	if (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
		ROS_INFO_NAMED("tutorial", "Visualizing plan 3 (with constrains) success");
	}
	else {
		ROS_INFO_NAMED("tutorial", "Visualizing plan 3 (with constrains) failed");
	}

	move_group.move();
	move_group.clearPathConstraints();
	return 0;
}
