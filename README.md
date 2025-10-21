# SGD-ICP_SLAM_Gnuplot
SGD-ICPを適用したSLAMです。
地図の描画はGnuplotを使ってます。
地図データがそのまま蓄積していく仕様になっています。

# 起動コマンド
※各ターミナルで実行するのを忘れないでください。
```
source ~/catkin_ws/devel/setup.bash
```
順番に起動してください。ターミナルを分けて

```
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```
```
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```
```
rosrun research LiDAR_node_SGD
```
```
rviz
```

