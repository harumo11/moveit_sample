#include <iostream>
#include <ros/ros.h>

//MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

//Eigen
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "robot_model_and_state_tutorial");
	ros::AsyncSpinner spinner(1);
	spinner.start();

	//チュートリアル開始
	//RobotModleクラスを使うときは，robot_model_loaderクラスを
	//使用して，そのクラスからシェアードポインターとしてRobotModelを
	//受け取ることが望ましいです．
	//RobotModelLoaderはROSのパラメータサーバからrobot descriptionを
	//探してきて，我々に与えてくれます．
	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
	//model frame : panda_link0
	ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

	//RobotModelを使用するにあたり，RobotStateと契約することができます．
	//RobotStateはロボットの設定を内包しています．
	//我々は全ての関節角度の状態をそれらのデフォルト値に設定します．
	//それにより，我々はJointModelGroupクラスを得ることができます．
	//このクラスは特定のモデルグループを表現しています．
	//たとえば，"panda_arm"とか
	robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
	kinematic_state->setToDefaultValues();
	const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("panda_arm");
	const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
	std::cout << "joint model group / variable names" << std::endl;
	//panda_joint1
	//panda_joint2
	//panda_joint3
	//panda_joint4
	//panda_joint5
	//panda_joint6
	//panda_joint7
	for (auto e : joint_names){
		std::cout << e << std::endl;
	}

	//
	//Get Joint Value
	//
	//我々はPnada robot のRobotStateに保存されている現在の関節角度を検索することができる
	std::vector<double> joint_values;
	kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
	//[ INFO] [1542615208.173979795]: Joint panda_joint1: 0.000000
	//[ INFO] [1542615208.173987599]: Joint panda_joint2: 0.000000
	//[ INFO] [1542615208.173994981]: Joint panda_joint3: 0.000000
	//[ INFO] [1542615208.174001228]: Joint panda_joint4: -1.570800
	//[ INFO] [1542615208.174007759]: Joint panda_joint5: 0.000000
	//[ INFO] [1542615208.174013180]: Joint panda_joint6: 0.000000
	//[ INFO] [1542615208.174019401]: Joint panda_joint7: 0.000000
	for (int i = 0; i < joint_names.size(); ++i) {
		ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
	}

	//
	//Joint Limits
	//
	//setJointGroupPosition()だけでは関節角度の限界を実施(変更?)しませんが
	//enforceBounds()を呼ぶことは，それを行います．
	
	//panda armの１関節を関節限界の外に設定します．
	joint_values[0] = 5.57;
	kinematic_state->setJointGroupPositions(joint_model_group, joint_values);

	//全ての関節角度が関節限界の中に存在するかをチェックします．
	//Should be invalid!
	if (!kinematic_state->satisfiesBounds()) {
		ROS_INFO("Joint angle are not valid, there is a outside of limit");
	}
	else {
		ROS_INFO("All joints are inside of limit");
	}

	//その関節限界を実施(変更?)し，もう一度チェックします．
	kinematic_state->enforceBounds();
	//Should be valid!
	if (!kinematic_state->satisfiesBounds()) {
		ROS_INFO("Joint angle are not valid, there is a outside of limit");
	}
	else {
		ROS_INFO("All joints are inside of limit");
	}

	
	//
	//Forward kinematics
	//
	//ランダムな関節の角度に対して，我々は順運動学を計算することが出来ます．
	//我々はpanda_link8の姿勢を見つけたいということに注意してください．
	//panda_groupの中でpanda_link8は最も遠方のリンクです．
	//各関節角度をランダムな値に変更　ー＞　joint_model_group(関節のつき方の情報を与える) ー＞　順運動学でどのリンクの姿勢を求めるのかを指定　ー＞　手先の姿勢を得る．
	kinematic_state->setToRandomPositions(joint_model_group);
	const Eigen::Affine3d end_effector_state = kinematic_state->getGlobalLinkTransform("panda_link8");
	//end-effectorの姿勢を表示する．これはモデルフレームの中にあるということに注意してください
	ROS_INFO("Forward kinematics");
	ROS_INFO_STREAM("Translation: \n" << end_effector_state.translation() << "\n");
	ROS_INFO_STREAM("Rotation: \n" << end_effector_state.rotation() << "\n");

	//
	//Inverse kinematics
	//
	//我々は逆運動学IKも解くことが可能です．
	//IKを解くためには我々は下記のことについて知っておく必要があります．
	// 
	// - end-effectorの所望の姿勢(デフォルトではpanda_armの最後のリンク)
	//   今回使用するend_effector_stateはすでに上で計算してあります．
	// - 試行回数(10)
	// - 各試行にかける時間(Timeout) 0.1s
	std::size_t attempt = 10;
	double timeout = 0.1;
	bool found_ik = kinematic_state->setFromIK(joint_model_group, end_effector_state, attempt, timeout);
	//もし見つかれば，それを表示することができます
	if (found_ik) {
		ROS_INFO("IK solution");
		kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
		for (int i = 0; i < joint_names.size(); i++) {
			ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
		}
	}
	else {
		ROS_INFO("Did not find IK solution");
	}

	//
	//Get the Jacobian
	//
	//ヤコビアンもRobotStateクラスから得ることが出来ます．
	Eigen::Vector3d reference_point_position(0.0,0.0,0.0);
	Eigen::MatrixXd jacobian;
	kinematic_state->getJacobian(joint_model_group, 
			                     kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
								 reference_point_position, jacobian);
	ROS_INFO_STREAM("Jacobian: \n" << jacobian << "\n");



	return 0;
}
