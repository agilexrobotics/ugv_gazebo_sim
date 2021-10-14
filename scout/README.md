[TOC]

# Scout仿真操作文档

## 一、功能包介绍

```
├── scout_control
├── scout_description
└── scout_gazebo_sim
```

​	scout_gazebo_sim：该文件夹为gazebo仿真功能包

​	scout_control: 该文件夹为仿真控制器功能包

​	scout_description: 该文件夹为模型文件功能包

## 二、环境

### 开发环境

​	ubuntu 18.04 + [ROS Melodic desktop full](http://wiki.ros.org/melodic/Installation/Ubuntu)

### 下载安装必备功能包

​	下载安装ros-control功能包，ros-control是ROS提供的机器人控制中间件

```
sudo apt-get install ros-melodic-ros-control
```

​	下载安装ros-controllers功能包，ros-controllers是ROS提供的常见车型运动学插件

```
sudo apt-get install ros-melodic-ros-controllers
```

​	下载安装gazebo-ros功能包，gazebo-ros是gazebo和ROS之间的通信接口，将ROS和Gazebo连接起来

```
sudo apt-get install ros-melodic-gazebo-ros
```

​	下载安装gazebo-ros-control功能包， gazebo-ros-control是在ROS和Gazebo之间通信的标准控制器

```
sudo apt-get install ros-melodic-gazebo-ros-control
```

​	下载安装teleop-twist-keyboard 功能包，teleop-twist-keyboard是键盘控制功能包，可以通过键盘上的“i”，“j”，“l”，“，”控制机器人前进，向左，向右，后退

```
sudo apt-get install ros-melodic-teleop-twist-keyboard 
```



## 三、用法

### 1、创建工作空间、下载仿真模型功能包并编译

​		打开一个新终端，创建一个工作空间，名字为scout_ws，在终端中输入：

```
mkdir scout_ws
```

​		进入到scout_ws文件夹中

```
cd scout_ws
```

​		创建一个用于存放功能包的文件夹，名字为src

```
mkdir src
```

​		进入到src文件夹

```
cd src
```

​		初始化文件夹

```
catkin_init_workspace
```

​		下载仿真模型功能包

```
git clone https://github.com/agilexrobotics/ugv_sim/scout.git
```

​		进入scout_ws文件夹

```
cd scout_ws
```

​		确认功能包的依赖有没有安装好

```
rosdep install --from-paths src --ignore-src -r -y 
```

​		进行编译

```
catkin_make
```



### 2、运行scout_v2和scout_mini的启动文件，在Rviz中可视化urdf文件

​	进入到scout_ws文件夹

```
cd scout_ws
```

​	声明环境变量

```
source devel/setup.bash
```

​	运行scout_v2的模型启动文件，在Rviz中可视化模型

```
roslaunch scout_description display_scout_v2.launch 
```

![img](image/scoutv2_rviz.png) 

​	运行scout_mini的模型启动文件，在Rviz中可视化模型

```
roslaunch scout_description display_scout_mini.launch 
```

![img](image/scout_mini_rviz.png) 

### 3、启动scout_v2和scout_mini的仿真环境，并在gazebo中控制scout_v2和scout_mini运动

​	进入到scout_ws文件夹

```
cd scout_ws
```

​	声明环境变量

```
source devel/setup.bash
```

​	启动scout_v2的仿真环境

```
roslaunch scout_gazebo_sim scout_empty_world.launch
```

![img](image/scoutv2_gazebo.png) 

#键盘控制，启动键盘控制之后，可以通过“i”，“j”，“l”，“，”控制scout2.0和scout_mini，前进，向左，向右，后退

```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py 
```

![img](image/teleop.png) 

​	启动scout_mini的仿真环境

```
roslaunch scout_gazebo_sim scout_mini_playpen.launch
```

![img](image/scout_mini_gazebo.png) 

#键盘控制，启动键盘控制之后，可以通过“i”，“j”，“l”，“，”控制scout2.0和scout_mini，前进，向左，向右，后退

```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py 
```

![img](image/teleop.png) 



 

