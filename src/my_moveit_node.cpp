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


	//
	//表示
	//
	
	//MoveItVisualizToolsパッケージは多くの物体をRviz上に表示させることができます．
	//たとえば，物体，ロボット，そしてトラジェクトリーです．
	//さらにデバッグ用にステップバイステップですすめることもできます．
	
	namespace rvt = rviz_visual_tools;
	moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
	visual_tools.deleteAllMarkers();

	//remoteControlはユーザーにGUIによるステップバイステップの
	//制御を提供するツールです．Rvizにてボタンやキーボードからの命令を受け付けます．
	visual_tools.loadRemoteControl();

	//Rvizは多くのマーカーを提供します．このデモではテキスト，円柱，球体を利用します．
	Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
	text_pose.translation().z() = 1.75;
	visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

	//バッチパブリッシュはRvizに送られるメッセージの数を減らします．
	visual_tools.trigger();
	
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
	//デモ開始
	//
	//ターミナルに出力
	visual_tools.prompt("Press 'next' in the RvizVisualToolsGUI window to start the demo");
	
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
	
	//move_group.move();

	//プランの表示
	//我々はプランをマーカのラインとしてRviz上に表示できます．
	ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
	visual_tools.publishAxisLabeled(target_pose1, "pose1");
	visual_tools.publishText(text_pose, "Pose goal", rvt::WHITE, rvt::XLARGE);
	visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
	visual_tools.trigger();
	visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
	

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

	//表示
	visual_tools.deleteAllMarkers();
	visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
	visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
	visual_tools.trigger();
	visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

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
	//Oeientationだけ変更.その他は現在のstart_stateのまま
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

	//表示
	visual_tools.deleteAllMarkers();
	visual_tools.publishAxisLabeled(start_pose2, "start");
	visual_tools.publishAxisLabeled(target_pose1, "goal");
		visual_tools.publishText(text_pose, "Gonstraind Goal", rvt::WHITE, rvt::XLARGE);
	visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
	visual_tools.trigger();
	visual_tools.prompt("next step");

	//move_group.move();
	move_group.clearPathConstraints();


	//
	//直交座標系のパス
	//
	
	//あなたは直交座標系空間でパスプランニングをwaypointsを設定することによって
	//直接的に行うことが可能です．waypointsとはアームのエンドエフェクタが通りすぎる
	//点のことです．
	//我々が上述の新しい状態からスタートしていることに気をつけてください．
	//初期ポーズ（start state)はwaypointのリストに加える必要はありません．
	//しかし，加えたほうがビジュアライズの助けにはなります．
	
	std::vector<geometry_msgs::Pose> waypoints;
	waypoints.push_back(start_pose2);

	geometry_msgs::Pose target_pose3 = start_pose2;;

	target_pose3.position.z -= 0.2;
	waypoints.push_back(target_pose3);	//down

	target_pose3.position.y -= 0.2;
	waypoints.push_back(target_pose3);	//right

	target_pose3.position.z += 0.2;
	target_pose3.position.y += 0.2;
	target_pose3.position.x -= 0.2;
	waypoints.push_back(target_pose3);	//up and left

	//直交座標系でのしばしば，ゆっくりとした動作が必要とされます．
	//アプローチや把持の再試行といった動作にです．
	//ここで，我々はスケーリングファクターを利用してロボットの
	//各関節の最大動作速度を少なくする方法を勉強します．
	//エンドエフェクターのスピードでは無いことに注意してください．
	move_group.setMaxVelocityScalingFactor(0.1);

	//我々は１ｃｍずつ補間された直交座標系のパスが欲しいので
	//最大ステップを0.01に指定しましょう．
	//我々はジャンプしきい値を0.0に設定します．そうすることで
	//ジャンプしきい値を無効にすることができます．
	//実際のハードウェアを操作している間にジャンプしきい値を無効にすると
	//予期せぬ余分なジョイントの動きが発生し，安全上の問題になることがあります．
	
	moveit_msgs::RobotTrajectory trajectory;
	const double jump_threshold = 0.0;
	const double eef_step = 0.01;
	double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
	//move_group.move();
	ROS_INFO_NAMED("tutorial", "visualizing plan(Cartesian path)", fraction * 100.0);

	//表示
	visual_tools.deleteAllMarkers();
	visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
	visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
	for (int i = 0; i < waypoints.size(); ++i) {
		visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::MEDIUM);
	}
	visual_tools.trigger();
	visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

	return 0;
}
