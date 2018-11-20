# My moveit

個人的なMoveIt!の練習

- my__moveit_node.cpp
	move_group_interfaceの練習．手先の移動とかwaypointを指定しての
	軌道の設定などを行う．

- robot__state.cpp
	robot_stateクラスとrobot_modelクラスを使って何ができるかの練習．
	逆運動学とかヤコビアンとか現在の関節角度とか取れる．


- planning_scene.cpp

## my_moveit_node.cpp

説明

## robot_stat.cpp

説明

## planning_scene.cpp
PlanningSceneクラスは衝突検出と制約（Constraint）検出の主なインターフェースを提供します．
PlanningSceneクラスはRobotModelクラスもしくは，URDF&SRDFから設定できます．
しかし，これらはおすすめの方法ではありません．おすすめの方法はPlanningSceneMonitorを
使用することです．

