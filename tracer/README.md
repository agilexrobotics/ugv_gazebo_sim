[TOC]

# tracer仿真操作流程

## 一、功能包介绍

```
├── tracer_description
└── tracer_gazebo_sim
```

tracer_description: 该文件为模型文件功能包

tracer_gazebo_sim: 该文件夹为gazebo仿功能包

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

​		打开一个新终端，创建一个工作空间，名字为tracer_ws，在终端中输入：

```
mkdir tracer_ws
```

​		进入到tracer_ws文件夹中

```
cd tracer_ws
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
git clone https://github.com/agilexrobotics/ugv_sim/tracer.git
```

​		进入tracer_ws文件夹

```
cd tracer_ws
```

​		确认功能包的依赖有没有安装好

```
rosdep install --from-paths src --ignore-src -r -y 
```

​		进行编译

```
catkin_make
```



### 2、运行tracer的启动文件，在Rviz中可视化urdf文件

​	进入到tracer_ws文件夹

```
cd tracer_ws
```

​	声明环境变量

```
source devel/setup.bash
```

​	运行tracer的模型启动文件，在Rviz中可视化模型

```
roslaunch tracer_description display_models.launch 
```

![img](image/rviz.png) 

### 3、运行tracer_gazebo_sim的启动文件，并在gazebo中控制tracer运行

​	进入到tracer_ws文件夹

```
cd tracer_ws
```

​	声明环境变量

```
source devel/setup.bash
```

​	启动tracer的仿真环境

```
roslaunch tracer_gazebo_sim tracer_playpen.launch
```

![img](image/gazebo.png) 

​	键盘控制，启动键盘控制之后，可以通过“i”，“j”，“l”，“，”控制scout2.0和scout_mini，前进，向左，向右，后退

```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py 
```

![img](image/teleop.png) 

