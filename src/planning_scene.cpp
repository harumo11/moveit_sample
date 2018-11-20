#include <iostream>

//ros
#include <ros/ros.h>

//MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>

#include <moveit/kinematic_constraints/utils.h>

//サブチュートリアル
//PlanningSceneクラスにユーザー定義のConstrainを指定することもできます．
//これはsetStateFeasibilityPredicate関数をもちいてコールバック関数を設定することで
//行われます．これはユーザー定義の簡単なコールバック関数の例です．
//このコールバック関数ではpanda_joint1が正の角度か，負の角度にあるかを判定します．
//[feasibility]->[実行可能性]
bool stateFeasibilityTestExample(const robot_state::RobotState& kinematic_state, bool verbose)
{
	const double* joint_value = kinematic_state.getJointPositions("panda_joint1");
	if (joint_value[0] > 0.0) {
		return true;
	}
	else {
		return false;
	}
}


int main(int argc, char* argv[])
{
	ros::init(argc, argv, "panda_arm_kinematics");
	ros::AsyncSpinner spinner(1);
	spinner.start();
	std::size_t count = 0;


	//PlanningSceneクラスはRoboModelクラスから簡単に導入できます．
	//しかし，これはおすすめの方法ではありません．
	//おすすめはPlanningSceneMonitorメソッドを使ってインスタンス化する法が
	//おすすめですが，ここでは紹介しません．
	robot_model_loader::RobotModelLoader robot_model_loader_("robot_description");
	robot_model::RobotModelPtr kinematic_model = robot_model_loader_.getModel();
	planning_scene::PlanningScene planning_scene_(kinematic_model);

	//
	//Collision checking
	//
	//self-collision check
	//まずはじめに，ロボットが自身の身体でself-collisionを起こしていないか
	//をチェックします．これを行うために，CollisionRequestオブジェクトと
	//CollisionResultオブジェクトを生成し，衝突検知関数をそれらに渡します．
	//self-collisionしているかどうかの調査結果はresultに含まれていることに気をつけてください．
	collision_detection::CollisionRequest collision_request_;
	collision_detection::CollisionResult collision_result_;
	planning_scene_.checkSelfCollision(collision_request_, collision_result_);
	ROS_INFO_STREAM("Test 1 Current state is : " << (collision_result_.collision ? "in" : "not in") << "self collision");

	//
	//Change the state
	//
	//では，現在のロボットの状態を変えてみましょう．
	//planning_sceneは現在の状態を永遠に維持します．
	//我々はそれを参照することも出来ますし，変更することもできます．
	//そして，新しいロボットの設定に対して衝突検知をすることも出来ます．
	//我々はcollision_resultをきれいにしなければなりません．新たな
	//collision_requestを作る前に．
	robot_state::RobotState& current_state = planning_scene_.getCurrentStateNonConst();
	current_state.setToRandomPositions();
	collision_result_.clear();
	planning_scene_.checkSelfCollision(collision_request_, collision_result_);
	ROS_INFO_STREAM("Test 2: Current state is " << (collision_result_.collision ? "in" : "not in") << " self collision");
	

	//
	//Getting contact information
	//
	//まずはじめに，手動でpanda armを自己衝突状態へ移動させましょう．
	//この状態は，実際は関節の稼働領域の外側ということに注意しましょう．
	std::vector<double> joint_values = {0.0, 0.0, 0.0, -2.9, 0.0, 1.4, 0.0};
	const robot_model::JointModelGroup* joint_model_group = current_state.getJointModelGroup("panda_arm");
	current_state.setJointGroupPositions(joint_model_group, joint_values);
	ROS_INFO_STREAM("Test 4: Current state is " << (current_state.satisfiesBounds(joint_model_group) ? "valid" : "not valid"));
	//もうすでに，衝突情報を得ることができます．collision_requestの然るべき部分を入力し,
	//返ってくる接触の最大数を設定したら
	//問い合わせてみましょう．
	collision_request_.contacts = true;
	collision_request_.max_contacts = 1000;

	collision_result_.clear();
	planning_scene_.checkSelfCollision(collision_request_, collision_result_);
	ROS_INFO_STREAM("Test 5: Current state is " << (collision_result_.collision ? "in" : "not in") << "self collision");
	collision_detection::CollisionResult::ContactMap::const_iterator it;
	for (it = collision_result_.contacts.begin() ; it != collision_result_.contacts.end(); ++it){
		ROS_INFO("Contact between: %s and %s", it->first.first.c_str(), it->first.second.c_str());
	}
	return 0;
}
