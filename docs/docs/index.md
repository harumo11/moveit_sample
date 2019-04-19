# MoveIt!のチュートリアル

!!! Source
	[ここの翻訳](https://ros-planning.github.io/moveit_tutorials/index.html)

!!! 参考
	[material cheet sheet](https://yakworks.github.io/mkdocs-material-components/cheat-sheet/)

これらのチュートリアルを通してMoveIt!を学びましょう．ちなみに，MoveIt!はモーションプランニングフレームワークですよ．

![image](https://ros-planning.github.io/moveit_tutorials/_images/rviz_plugin_head.png)

これらのチュートリアルでは，Franka Emika Pandaロボットを例と指定使用します．
すでにMoveIt!に対応する用に設定されたロボットであれば，簡単にPandaロボットを置き換えることが
可能です．ロボットが**すぐに**利用可のかどうかは[MoveIt!ですぐに使用可能な
ロボットのリスト](https://moveit.ros.org/robots/)を見てください．もし，リストにあなたのお気に入りの
ロボットが無い場合には，あなた自身がMoveIt!の設定を作成すれば，あなたのロボットでMoveIt!を
使用することができるようになります．
詳しくは"新しいロボットで使うには"の章を見てください．

!!! Note
	これはROS MelodicバージョンのMoveIt!の和訳です．Kineticのチュートリアルは
	[Kinetic 英語](http://docs.ros.org/kinetic/api/moveit_tutorials/html/index.html)を見てください．

## ここから始めよう！ MoveIt!とRviz

- [ここから始めよう！](1.md)
- [MoveIt!のクイックスタート．Rvizとともに](2.md)

## MoveGroup - C++とPythonのROSラッパー - 
MoveIt!を使用する最も簡単な方法は`move_group_interface`クラスを利用することです．
このインターフェースを利用することは初心者におすすめです．さらに，このクラスはMoveIt!の
多くの機能へ統一された方法でアクセスすることができます．

- Move Group C++ インターフェース
- Move Group Python インターフェース
- MoveIt! Commander インターフェース

## C++を利用して直接的にMoveIt!を使う
MoveIt!を利用して複雑なアプリケーションを作成するときにはMoveIt!のC++インターフェースを
利用して下さい．さらに，直接的にC++ APIを使用することにより多くのROSサービス/アクション層を
スキップできるので，パフォーマンスが大幅に向上します．

- `Robot Model`と`Robot State`
- プランニングシーン(Planning Scene)
- プランニングシーン(ROS API)
- モーションプランニングAPI
- モーションプランニングのパイプライン
- 衝突の表示
- 時間パラメータの設定
- ゆるく組み合わさった拘束とプランニング
- ピック＆プレイス

## 新しいロボットで使うには
新しいロボットのMoveIt!の設定を作成する前に，[このリスト](https://moveit.ros.org/robots/)で
そのロボットが設定済みであるかどうかを確認してください．なかった場合にはこのチュートリアルを
参照してください.(そして，その結果をMoveIt!のメーリングリストに投稿してください)

- MoveIt!セットアップアシスタント
- `URDF`と`SRDF`
- 低レベルコントローラ
- 知覚パイプラインのチュートリアル
- `IKfast`ソルバー
- `Track-IK`ソルバー

## 設定

- 運動学の設定
- Fakeコントローラマネージャー
- カスタム制約サンプラー
- OMPLプランナー
- CHOMPプランナー
- STOMPプランナー
- プランニングアダプターのチュートリアル

## その他

- ジョイスティック
- ベンチマーク
- 結合/ユニット テスト
