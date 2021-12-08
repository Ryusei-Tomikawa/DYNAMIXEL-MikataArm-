# mikata-arm
mikata-armのROSパッケージ
既存のmikata-armの6軸のパッケージ[Mikata-arm Ros パッケージ](https://github.com/ROBOTIS-JAPAN-GIT/open_manipulator/tree/dynamixel_6dof_mikata_arm)は
Gazebo+Moveit!は可能だが、moveit_commanderを用いたプログラムを用いるとうまく動作しないことがわかった

よって、ここのリポジトリを用いることがシミュレーションでも実機環境でも動作可能にできるようにパッケージを変更した

簡易的ではあるが、サンプルコードも入っているので実際に試してみると動作確認できると思われる

# インストール方法
上記のMikata-arm Ros パッケージを参考にすればよいが、下記までgit clone すること
 ```shell
    $ git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
    $ git clone https://github.com/ROBOTIS-GIT/dynamixel-workbench.git
    $ git clone https://github.com/ROBOTIS-GIT/dynamixel-workbench-msgs.git
    $ git clone https://github.com/ROBOTIS-GIT/robotis_manipulator.git
    $ git clone https://github.com/ROBOTIS-GIT/open_manipulator_msgs.git
```


```shell
    $ cd ~/catkin_ws/src
    $ git clone https://github.com/Ryusei-Tomikawa/mikata-arm
    $ cd ..
    $ catkin build
```
 
これで一応環境開発は整うはず

# 動作方法

基本的にはOpen_manipulatorと同じなので。e-Manualを参考にするように[e-Manual](https://emanual.robotis.com/docs/en/platform/openmanipulator_x/quick_start_guide/)

これはシミュレーション環境で行う場合のコマンドである
 ``` shell
   $ sudo chmod 666 /dev/ttyUSB0
   $ roslaunch open_manipulator_controller open_manipulator_controller.launch use_gazebo:=true use_moveit:=true
 ```
 
 これは、実機環境で行う場合のコマンドである
  ``` shell
   $ sudo chmod 666 /dev/ttyUSB0
   $ roslaunch open_manipulator_controller open_manipulator_controller.launch use_moveit:=true
 ```
 
 USBの実行権限に関してのPathは適宜変更すること
 
 サンプルコードを動かすと、簡単なPick and Place動作を行う（グリッパー部分は、現状サービスで実装してあるが、本当はplanning nameのところにgripperを用いて、gripper controllerを使って開閉を行うのが普通なはず...?）
シミュレーション環境や実機環境どちらで動かしてもよいので、下記のコマンドで動作確認を行うとよい

 ```shell
   $ rosrun open_manipulator_moveit open_manipulator.py
 ```
 
# なぜうまくmoveit_comannderが動かないのか？
 open_manipulator_controller.launchを見てもらえばわかるが、既存のmikata-armのパッケージには、なぜが ns="mikata-arm"がgazeboやMoveit!を立ち上げるファイルに書かれている.
 namespaceが入ってしまうと、トピック名やサービス名全部が変わってしまい、joint_state_controllerやrobot_state_publisherの部分でうまくトピックを受け取ることができなくなり、
 tfを発行できなくなることにより、Rvizと実機のロボットが対応していないことになっている.
 
 例: 既存のパッケージを用いると、/joint_statesのトピック名の部分が、/mikata-arm/joint_statesとなっている. こうなってしまうと,robot_state_publisherでは,subscriberでは/joint_states
 というトピック名なので、受け取ることができない→したがって、namespaceをいらないところは消す必要があった.
