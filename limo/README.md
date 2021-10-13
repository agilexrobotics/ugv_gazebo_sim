limo仿真操作流程

一、功能包介绍

![img](image/jieshao.png)

limo_description: 该文件为模型文件功能包

limo_gazebo_sim: 该文件夹为gazebo仿功能包

二、环境

开发环境：ubuntu 18.04 + ROS Melodic desktop full。

假如你没有安装ROS，请按照官网的教程安装ROS。

ROS安装完成之后，按照下面的指令下载依赖：

sudo apt-get install ros-melodic-ros-control

sudo apt-get install ros-melodic-ros-controllers

sudo apt-get install ros-melodic-gazebo-ros

sudo apt-get install ros-melodic-gazebo-ros-control

sudo apt-get install ros-melodic-teleop-twist-keyboard	

sudo apt-get install ros-melodic-rqt-robot-steering 

三、用法

第一步、创建工作空间并下载功能包

打开一个新终端，创建一个新的工作空间，在终端中输入：

mkdir limo_ws/src

cd limo_ws/src

catkin_init_workspace

git clone 

cd ..

rosdep install --from-paths src --ignore-src -r -y  

catkin_make

第二步、运行limo的启动文件，在Rviz中可视化urdf文件

cd limo_ws

source devel/setup.bash

\#启动limo

roslaunch limo_description display_models.launch 

![img](image/rviz.png) 

第三步、运行limo_gazebo_sim的启动文件，并在gazebo中控制limo运动

cd limo_ws

source devel/setup.bash

\#启动limo仿真环境

\#阿克曼运动模式

roslaunch limo_gazebo_sim limo_ackerman.launch

rosrun rqt_robot_steering rqt_robot_steering

![img](image/limo_ackerman.png) 

通过Robot Steering 小插件控制limo运动

\#四轮差速运动模式

roslaunch limo_gazebo_sim limo_four_diff.launch 

\#键盘控制

rosrun teleop_twist_keyboard teleop_twist_keyboard.py 

![img](image/limo_diff.png) 

 

 
