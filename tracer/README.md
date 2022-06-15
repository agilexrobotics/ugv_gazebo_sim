[TOC]

# Tracer Simulation Operation Process

## 1.	Introduction of Function Package

```
├── tracer_description
└── tracer_gazebo_sim
```

tracer_description: The file is the function package of model file

tracer_gazebo_sim: The folder is gazebo simulation function package

## 2.	Environment

### Development Environment

​	ubuntu 18.04 + [ROS Melodic desktop full](http://wiki.ros.org/melodic/Installation/Ubuntu)

### Download and install required function package

​	Download and install ros-control function package, ros-control is the robot control middleware provided by ROS

```
sudo apt-get install ros-melodic-ros-control
```

​	Download and install ros-controllers function package, ros-controllers are the kinematics plug-in of common models provided by ROS

```
sudo apt-get install ros-melodic-ros-controllers
```

​	Download and install gazebo-ros function package, gazebo-ros is the communication interface between gazebo and ROS, and connect the ROS and Gazebo

```
sudo apt-get install ros-melodic-gazebo-ros
```

​	Download and install gazebo-ros-control function package, gazebo-ros-control is the communication standard controller between ROS and Gazebo

```
sudo apt-get install ros-melodic-gazebo-ros-control
```

​	Download and install joint-state-publisher-gui package.This package is used to visualize the joint control.

```
sudo apt-get install ros-melodic-joint-state-publisher-gui 
```

​	Download and install teleop-twist-keyboard function package, telop-twist-keyboard is keyboard control function package, the robot can be controlled to move forward, left, right and backward through "i", "j", "l",and "," on the keyboard

```
sudo apt-get install ros-melodic-teleop-twist-keyboard 
```



## 3.	About Usage

### 1.	Create workspace, download simulation model function package and compile

​	Open a new terminal and create a workspace named tracer_ws, enter in the terminal:

```
mkdir tracer_ws
```

​		Enter the tracer_ws folder

```
cd tracer_ws
```

​		Create a folder to store function package named src
```
mkdir src
```

​		Enter the src folder

```
cd src
```

​		Initialize folder

```
catkin_init_workspace
```

​		Download simulation model function package

```
git clone https://github.com/agilexrobotics/ugv_sim.git
```

​		Enter the tracer_ws folder

```
cd tracer_ws
```

​		Confirm whether the dependency of the function package is installed

```
rosdep install --from-paths src --ignore-src -r -y 
```

​		Compile

```
catkin_make
```



### 2.	Run the star file of tracer model and visualize the urdf file in Rviz

​	Enter the tracer_ws folder

```
cd tracer_ws
```

​	Declare the environment variable

```
source devel/setup.bash
```

​	Run the start file of tracer and visualize the model in Rviz

```
roslaunch tracer_description display_models.launch 
```

![img](image/rviz.png) 

### 3.	Run the start file of tracer_gazebo_sim and control tracer movement in gazebo

​	Enter the tracer_ws folder

```
cd tracer_ws
```

​	Declare the environment variable

```
source devel/setup.bash
```

​	Start the simulation environment of tracer

```
roslaunch tracer_gazebo_sim tracer_playpen.launch
```

![img](image/gazebo.png) 

​	Control by keyboard, the robot can be controlled to move forward, left, right and backward through "i", "j", "l",and "," on the keyboard

```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py 
```

![img](image/teleop.png) 

