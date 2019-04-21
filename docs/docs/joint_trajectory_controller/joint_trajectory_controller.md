# joint_trajectory_controller

!!! Quote
	- [joint_trajectory_controller](http://wiki.ros.org/joint_trajectory_controller)
	- [UnderstandingTrajectoryReplacement](http://docs.ros.org/api/trajectory_msgs/html/msg/JointTrajectory.html)
	- [trajectory_msgs/JointTrajectoryMessage](http://docs.ros.org/api/trajectory_msgs/html/msg/JointTrajectory.html)

## 概要
ジョイントグループのジョイント空間での軌道(trajectory)を実行するコントローラである．
軌道は，到達時刻が指定された経由点(waypoint)の集合によって指定されます．そして，
コントローラは，その機構が許す限りそれらの軌道を実行するように試みます．
経由点は位置(角度)，速度，加速度によって構成されます．この内，速度と加速度は指定しなくても
良いです．

### 軌道の置き換え
コントローラは複数の軌道補間で動作するように設定されています．デフォルトでは
スプライン補間が適用されます．しかし，他の軌道補間も利用可能です．
適用されるスプライン補間は，下記のように経由点の指定方法によって異なります．

[１次スプライン補間](https://ja.wikipedia.org/wiki/%E5%8C%BA%E5%88%86%E7%B7%9A%E5%BD%A2%E9%96%A2%E6%95%B0#/media/File:Finite_element_method_1D_illustration1.png)
:	もし，ジョイントの位置だけが指定された場合，この補間が指定されます．この場合，位置レベルでの連続は保証されます．
	しかし，推奨しません．なぜなら，速度の観点から見ると経由点は連続ではないからです．ここで言う連続は微分可能であるということと同義です．

[３次スプライン補間](https://help.xlstat.com/customer/ja/portal/articles/2062403-xlstat%E3%81%AB%E3%82%88%E3%82%8B3%E6%AC%A1%E3%82%B9%E3%83%97%E3%83%A9%E3%82%A4%E3%83%B3%E3%81%AE%E9%81%A9%E5%90%88)
:	ジョイントの位置と速度が指定された場合は，この補間が適用されます．速度レベルでの連続性は保証されます．

５次スプライン補間
:	ジョイントの位置，速度，加速度が指定された場合に適用されます．加速度レベルでの連続性が保証されます．
	
### ハードウェアのインターフェース
このコントローラは複数のインターフェースで動作するように設定されています．
現在利用可能なジョイントのタイプは**position**, **velocity**,そして**effort**です．
positionジョイントを有している場合，コントローラは指定された所望のジョイント位置をそのまま伝達します．
velocity(effort)ジョイントの場合は，PIDループによって位置＋速度の追従誤差が反映されたコマンドが
伝達されます．
コントローラの設定は[ここ](http://wiki.ros.org/joint_trajectory_controller#Controller_configuration_examples)で
見ることができます．

上記の軌道補間の場合と同様に，新しいハードウェアのインターフェースに対しても
サポートすることも可能ですし，また，既存のサポートしているposition,
velocity,effortインターフェースをマッピングすることもできます(例えば，effortコマンドを生成するた油圧コントローラ）．

### その他の特徴

- リアルタイムセーフな実装
- ラッピング(conitinuous)なジョイントに対する適切な取扱
- システムのクロックの変更にロバスト 
	連続的に増加していかないクロックを有するシステム下においても
	キューに入れられている軌道の実行は，不連続にならない．(入れられた順に実行される)

## 軌道を送る

### 利用可能なインターフェース

コントロラーにコマンドを送るためにはに種類のインターフェースを利用することが可能です．
**action interface**と**topic interface**です．両方共
[trajectory_msgs/JointTrajectory](http://docs.ros.org/api/trajectory_msgs/html/msg/JointTrajectory.html)
メッセージを軌道を指定するために行います．そして，`allow_partial_joints_goal`がTrueでない場合には，
メッセージには*すべて(サブセットだけでなくすべての)*のジョイントのコントローラの名前
(controller.yamlで指定して，パラメータサーバに登録されている)を書き込む必要があります．

軌道を送る第１の方法は**action interface**を使うことです．そして，実行の途中経過を観測したい
場合はこちらを選択することが望ましいでしょう．Action goalsは実行される軌道を指定するだけでなく
経路やゴールとの誤差（tolerance)も指定することができます．
ゴールとの誤差が指定されていない場合には，この値はパラメータサーバに登録されている
ものを使用します（詳しくは
[ROS API](http://wiki.ros.org/joint_trajectory_controller#ROS_API)
を見てください）．
もし，軌道の実行中にゴールとの誤差を満たせなかった場合はactionは中止され，
クライアントにもそのことが通知されます．

!!! Note
	ゴールへの軌道が中止されたときでも，コントローラはできる限り軌道を実行しようと試みるでしょう．

**topic interface**は*実行と忘却(fire-and-forget)*を行う方法です．

!!! Note 
	fire-and-forgetとはミサイルの発射・誘導方式に関する軍事用語で，発車後にミサイル自体が
	標的を追尾する能力を持つため発射母体g誘導のために照準を持続させたり，他の手段による標的に
	誘導照準を行ったりする必要がないものを言う．

軌道の実行を監視する必要のないときにはこちらを使ってください．topic interfaceを使用する場合は
コントローラの経路やゴールとの誤差をしているすることはできません．なぜなら，
送信したノードに対してゴールとの誤差が大きいことを教える標準的な方法が無いからです．
標準的な方法が準備されてないだけで，観測の方法はいくつかあります．
例えば，query_stateサービスとstateトピック(詳しくは
[下記のROS APIの章](http://wiki.ros.org/joint_trajectory_controller#ROS_API)
を参照してください）を通して軌道の実行を観測することができます．
これは，ちょっと面倒な(しかし柔軟な)action interfaceの実現方法です．

### 中断(Preemption)戦略

ただ１つのアクションのみが有効なとき，もしくはtopic interfaceが使用されていないときには
，経路とゴールまでの誤差は，現在有効なその軌道に対してのみチェックされます．

現在有効な軌道が他のactionやtopicインターフェースからのコマンドによって横取りされたとき
現在のゴールはキャンセルされ，クライアントにもそれが通知されます．

topic interfaceから**空の軌道**を送ると，*すべて*の実行が予定されている軌道をキャンセルし
現在位置を保持するモードに入ります．
`stop_trajectory_duration`パラメータはこのストップモーションの実行にどのくらいの時間を
許すかを設定します．

### 軌道の置き換え

ジョイント軌道メッセージはいつ新しい軌道の実行を開始するかという時刻の設定ができます．
この機能はヘッダーのタイムスタンプによって実現します．ここに０（デフォルト）を入れることは
*すぐに実行すること*を意味します．

新しい軌道のコマンドの到着は必ずしもコントローラが感染に現在実行中の軌道を捨てさり，
代わりに新しい軌道を実行することを意味しません．
むしろ，コントローラは両方の軌道の便利な部分を取り，それらを適切に組み合わせます．
この振る舞いに関しての詳細は
[understanding trajectory replacement](http://wiki.ros.org/joint_trajectory_controller/UnderstandingTrajectoryReplacement)
を参照してください．

## ROS API
### 説明
#### 3.1.1 action interface

コントローラは
[control_msgs::FollowJointTrajectoryAction](http://docs.ros.org/api/control_msgs/html/action/FollowJointTrajectory.html)
インターフェースをコントローラの`follow_joint_trajectory`ネームスペース内に持っています．
何を送ればいいかについての情報に関してはactionの定義を見てください．

#### 3.1.2 subscribed topics
command([torajectory_msgs/JointTrajectory](http://docs.ros.org/api/trajectory_msgs/html/msg/JointTrajectory.html))
:	torajectory interfaceを利用して実行する軌道

#### 3.1.3 published topics
state([control_msgs/JointTrajectoryControllerState](http://docs.ros.org/api/control_msgs/html/msg/JointTrajectoryControllerState.html)
:	現在のコントローラの状態

#### 3.1.4 services
query_state([control_msgs/QueryTrajectoryState](http://docs.ros.org/api/control_msgs/html/srv/QueryTrajectoryState.html)
:	コントローラの今後の予定の問い合わせ

#### 3.1.5 parameters

- joints(string[])

	コントロールしている関節のリスト

- constraints/goal_time(double, default: 0.0) 

	もし，ゴール軌道上の点のタイムスタンプがtであり，t+/-`goal_time`以内に到着すれば，軌道の追従は成功となります．
	そうでなければ失敗となります．

- constraints/stopped_velocity_tolerance(double, default: 0.01)

	おおよそ０とみなしていい速度の設定

- constraints/<joint>/goal(double, default: 0.0)

	ゴールに到達したジョイントの位置の誤差に対する許容値．ジョイントの位置が
	goal_position+/-goal_tolerance以下にあるとき，その軌道は成功したものとみなされます．

- gains/<joint>(associative array)

	PIDコントローラの設定に関するキーと値のペア．この設定はeffortとvelocityインターフェースを
	利用しているときに必要とされます．具体的には
	[ここ](http://wiki.ros.org/joint_trajectory_controller#Controller_configuration_examples)
	を見てください．

- velocity_ff/<joint>(double, default: 0.0)
	
	フィードフォワード区間における速度ゲインで，PIDコントローラの出力に加えられます．
	これは任意であり，effortかvelocityインターフェースでのみしよされます．
	具体例は
	[ここ](http://wiki.ros.org/joint_trajectory_controller#Controller_configuration_examples)
	を見てください．

- stop_trajectory_duration(double, default: 0.0)

	コントローラの開始時や軌道をキャンセルしたときに，位置保持モード(position hold mode)に
	入ります．このパラメータによって実行中の状態（位置＆速度）が停止状態に移行するまでの時間を
	制御できます．この設定には０かそれ以上の値を入力してください．

- state_publish_rate(double, default: 50)

	どのくらいの頻度(Hz)でコントローラが状態を発信するかの設定

- action_monitor_rate(double, default: 20)

	action goal 状態を監視する頻度．この変数の変更は上級者向けなため，変更はおすすめできません．

- allow_partial_joint_goal(bool, defalut: False)

	一部のジョイントに対してのみの`JointTrajctory`メッセージの送信を許可します．

### コントローラの設定例 

#### 3.2.1 最小の設定例・position interface

```
head_controller:
  type: "position_controllers/JointTrajectoryController"
  joints:
    - head_1_joint
    - head_2_joint
```

#### 3.2.2 最小の設定例・velocity interface

```
head_controller:
  type: "velocity_controllers/JointTrajectoryController"
  joints:
    - head_1_joint
    - head_2_joint

  gains: # Required because we're controlling a velocity interface
    head_1_joint: {p: 100,  d: 1, i: 1, i_clamp: 1}
    head_2_joint: {p: 100,  d: 1, i: 1, i_clamp: 1}
```

#### 3.2.3 速度フィードフォワード・velocity interface

```
head_controller:
  type: "velocity_controllers/JointTrajectoryController"
  joints:
    - head_1_joint
    - head_2_joint

  gains: # Required because we're controlling a velocity interface
    head_1_joint: {p: 10,  d: 1, i: 1, i_clamp: 1} # Smaller 'p' term, since ff term does most of the work
    head_2_joint: {p: 10,  d: 1, i: 1, i_clamp: 1} # Smaller 'p' term, since ff term does most of the work

  velocity_ff:
    head_1_joint: 1.0
    head_2_joint: 1.0
```

#### 3.2.4 最小の設定例・effort interface

```
head_controller:
  type: "effort_controllers/JointTrajectoryController"
  joints:
    - head_1_joint
    - head_2_joint

  gains: # Required because we're controlling an effort interface
    head_1_joint: {p: 100,  d: 1, i: 1, i_clamp: 1}
    head_2_joint: {p: 100,  d: 1, i: 1, i_clamp: 1}
```

#### 3.2.5 完全な設定例・effort interface

```
head_controller:
  type: "effort_controllers/JointTrajectoryController"
  joints:
    - head_1_joint
    - head_2_joint

  constraints:
    goal_time: 0.5                   # Override default
    stopped_velocity_tolerance: 0.02 # Override default
    head_1_joint:
      trajectory: 0.05               # Not enforced if unspecified
      goal: 0.02                     # Not enforced if unspecified
    head_2_joint:
      goal: 0.01                     # Not enforced if unspecified

  gains: # Required because we're controlling an effort interface
    head_1_joint: {p: 100,  d: 1, i: 1, i_clamp: 1}
    head_2_joint: {p: 100,  d: 1, i: 1, i_clamp: 1}

  state_publish_rate:  25            # Override default
  action_monitor_rate: 30            # Override default
  stop_trajectory_duration: 0        # Override default
```
