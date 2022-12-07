# Visual-servo
# 环境
ubuntu 20.04

ros noetic
# 使用
启动仿真
```
roslaunch ur_gazebo ur10_chessboard.launch 
```
将ur10移动到合适位置，然后运行
```
rosrun visual_servo_kalman setgoal //设置目标图像
```
将ur10移动到另一位置，然后运行
```
rosrun visual_servo_kalman kalman_6dof //基于kalman滤波进行控制
```
![1](https://user-images.githubusercontent.com/13638834/168751371-f7cd68ef-4f8f-4a96-805d-a31634e917ea.png)
![111](https://user-images.githubusercontent.com/13638834/168751378-cd8c5231-8350-4097-86f0-813a8f1ac52a.png)
