[TOC]

# Hunter 2.0 Simulation Operation Process

## 1.Introduction of Function Package

```
├── hunter2_base
├── hunter2_gazebo
```

​	hunter2_base: The folder is model file function package

​	hunter2_gazebo: The file is Gazebo simulation function package

## 2.Environment

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



## 3.About Usage

### 	1.Create workspace, download simulation model function package and compile

​		Open a new terminal and create a workspace named hunter_ws, enter in the terminal:

```
mkdir hunter_ws
```

​		Enter the hunter_ws folder

```
cd hunter_ws
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

​		Enter the hunter_ws folder

```
cd hunter_ws
```

​	Compile

```
colcon build
```

### 	2.Run the star file of hunter2.0 model and visualize the model in Rviz

​		Enter the hunter_ws folder

```
cd hunter_ws
```

​		Declare the environment variable

```
source install/setup.bash 
```

​		Run the start file of hunter2.0 and visualize the model in Rviz

```
roslaunch hunter2_base display_xacro.launch
```

![说明文字](image/rviz.png)

### 	3.Start the gazebo simulation environment of hunter2.0 and control hunter2.0 movement in the gazebo

​		Enter the hunter_ws folder

```
cd hunter_ws
```

​		Declare the environment variable

```
source install/setup.bash 
source /usr/share/gazebo-11/setup.bash
```

​		Start the simulation environment of hunter2.0, slide the sliding bar of Robot Steering plug-in to control robot movement

```
ros2 launch hunter2_gazebo hunter_empty_world.launch.py 
```

![说明文字](image/gazebo.png)


