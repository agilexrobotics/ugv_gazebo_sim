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

​	Ubuntu 22.04  + [ROS Humble desktop full](http://docs.ros.org/en/humble/Installation.html)

### Download and install required function package

​		Download and install gazebo-ros function package, gazebo-ros is the communication interface between gazebo and ROS, and connect the ROS and Gazebo

```
sudo apt-get install ros-humble-gazebo-*
```

​	Download and install joint-state-publisher-gui joint-state-publisher package.This package is used to visualize the joint control.

```
sudo apt-get install ros-humble-joint-state-publisher ros-humble-joint-state-publisher-gui
```

​	Download and install ackermann-steering controller; The ackermann-steering controller is a gazebo plugin for controlling the car

```
sudo apt-get install ros-humble-ackermann-steering-controller
```

​	Download and install Control-related dependencies and feature packs; control is used to define the type of model joints

```
sudo apt-get install ros-humble-control-*
```

​	Download and install rqt-robot-steering plug-in, rqt_robot_steering is a ROS tool closely related to robot motion control, it can send the control command of robot linear motion and steering motion, and the robot motion can be easily controlled through the sliding bar

```
sudo apt-get install ros-humble-rqt-robot-steering 
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

​		Download simulation model function package

```
git clone https://github.com/agilexrobotics/ugv_sim.git
```

​		Enter the tracer_ws folder

```
cd tracer_ws
```

​		Compile

```
colcon build
```



### 2.	Run the star file of tracer model and visualize the urdf file in Rviz

​	Enter the tracer_ws folder

```
cd tracer_ws
```

​	Declare the environment variable

```
source install/setup.bash 
```

​	Run the start file of tracer and visualize the model in Rviz

```
ros2 launch tracer_description display_tracer.launch.py
```

![img](image/rviz.png) 

### 3.	Run the start file of tracer_gazebo_sim and control tracer movement in gazebo

​	Enter the tracer_ws folder

```
cd tracer_ws
```

​	Declare the environment variable

```
source install/setup.bash 
source /usr/share/gazebo-11/setup.bash
```

​	Start the simulation environment of tracer

```
ros2 launch tracer_gazebo_sim tracer_empty_world.launch.py 
```

![img](image/gazebo.png) 

​	Control by keyboard, the robot can be controlled to move forward, left, right and backward through "i", "j", "l",and "," on the keyboard

```
ros2 run teleop_twist_keyboard teleop_twist_keyboard 
```

![img](image/teleop.png) 

