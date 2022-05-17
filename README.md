# 使用
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
