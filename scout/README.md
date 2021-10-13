# Scout仿真操作文档

## 一、功能包介绍

![img](image/jieshao.png) 

scout_gazebo_sim：该文件夹为gazebo仿真功能包

scout_control: 该文件夹为仿真控制器功能包

scout_description: 该文件夹为模型文件功能包

## 二、环境

开发环境：ubuntu 18.04 + ROS Melodic desktop full。

假如你没有安装ROS，请按照官网的教程安装ROS。

ROS安装完成之后，按照下面的指令下载依赖：

```
sudo apt-get install ros-melodic-ros-control

sudo apt-get install ros-melodic-ros-controllers

sudo apt-get install ros-melodic-gazebo-ros

sudo apt-get install ros-melodic-gazebo-ros-control

sudo apt-get install ros-melodic-teleop-twist-keyboard		
```

## 三、用法

### 第一步、创建工作空间并下载功能包

打开一个新终端，创建一个新的工作空间，在终端中输入：

```
mkdir scout_ws/src

cd scout_ws/src

catkin_init_workspace

git clone 

cd ..

rosdep install --from-paths src --ignore-src -r -y  

catkin_make
```

### 第二步、启动scout的启动文件，在Rviz中可视化urdf文件

```
cd scout_ws

source devel/setup.bash

#启动scout 2.0

roslaunch scout_description display_scout_v2.launch 
```

![img](image/scoutv2_rviz.png) 

```
#启动scout_mini

roslaunch scout_description display_scout_mini.launch 
```

![img](image/scout_mini_rviz.png) 

### 第三步、启动scout_gazebo_sim的启动文件，并在gazebo中控制scout运动

```
cd scout_ws

source devel/setup.bash

#启动scout 2.0

roslaunch scout_gazebo_sim scout_empty_world.launch

#scout_empty_world.launch是打开一个只有scout2.0的空环境
```

![img](image/scoutv2_gazebo.png) 

```
#启动scout_mini

roslaunch scout_gazebo_sim scout_mini_playpen.launch

#scout_mini_playpen.launch 是打开一个具有多个障碍物的物理环境，可以用于后续的建图导航。
```

![img](image/scout_mini_gazebo.png) 

```
#键盘控制

rosrun teleop_twist_keyboard teleop_twist_keyboard.py 
```

![img](image/teleop.png) 

启动键盘控制之后，可以通过“i”，“j”，“l”，“，”控制scout2.0和scout_mini，前进，向左，向右，后退

 
