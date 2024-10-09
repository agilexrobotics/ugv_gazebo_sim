[TOC]

# Scout Simulation Operation Process

## 1.	Introduction of Function Package

```
├── scout_description
└── scout_gazebo_sim
```

​	scout_gazebo_sim：The folder is gazebo simulation function package

​	scout_description: The folder is the function package of model file

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

### 1、Create workspace, download simulation model function package and compile

​		Open a new terminal and create a workspace named scout_ws, enter in the terminal:

```
mkdir scout_ws
```

​		Enter the scout_ws folder

```
cd scout_ws
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

​		Enter the scout_ws folder

```
cd scout_ws
```

​	Compile
```
colcon build
```



### 2、Run the star file of scout_v2 and scout_mini, and visualize the urdf file in Rviz

​	Enter the scout_ws folder

```
cd scout_ws
```

​	Declare the environment variable

```
source install/setup.bash 
```

​	Run the start file of scout_v2 model and visualize the model in Rviz

```
ros2 launch scout_description display_scout_v2.launch.py 
```

![img](image/scoutv2_rviz.png) 

​	Run the start file of scout_mini model and visualize the model in Rviz

```
ros2 launch scout_description display_mini.launch.py
```

![img](image/scout_mini_rviz.png) 

### 3、Start the gazebo simulation environment of scout_v2 and scout_mini and control scout_v2 and scout_mini movement in the gazebo

​	Enter the scout_ws folder

```
cd scout_ws
```

​	Declare the environment variable

```
source install/setup.bash 
source /usr/share/gazebo-11/setup.bash
```

​	Start the simulation environment of scout_v2

```
ros2 launch scout_gazebo_sim scout_v2_empty_world.launch.py 
```

![img](image/scoutv2_gazebo.png) 

#Control by keyboard, the scout2.0 and scout_mini can be controlled to move forward, left, right and backward through "i", "j", "l",and "," on the keyboard

```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

![img](image/teleop.png) 

​	Start the simulation environment of scout_mini

```
ros2 launch scout_gazebo_sim scout_mini_empty_world.launch.py 
```

![img](image/scout_mini_gazebo.png) 

#Control by keyboard, the scout2.0 and scout_mini can be controlled to move forward, left, right and backward through "i", "j", "l",and "," on the keyboard

```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

![img](image/teleop.png) 



 

