#include <iostream>
#include <ros/ros.h>

//MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

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
	//setJointGroupPosition()だけでは関節角度の制限を強制しませんが
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

	//その関節限界を強制し，もう一度チェックします．
	kinematic_state->enforceBounds();
	//Should be valid!
	if (!kinematic_state->satisfiesBounds()) {
		ROS_INFO("Joint angle are not valid, there is a outside of limit");
	}
	else {
		ROS_INFO("All joints are inside of limit");
	}

	
	return 0;
}
