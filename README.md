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
