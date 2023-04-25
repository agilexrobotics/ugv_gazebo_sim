[TOC]

# Hunter 2.0 Simulation Operation Process

## 1.Introduction of Function Package

```
├── hunter2_base
├── hunter2_control
├── hunter2_gazebo
├── image
├── steer_bot_hardware_gazebo
└── steer_drive_controller
```

​	hunter2_base: The folder is model file function package

​	hunter2_control: The file is simulation controller function package

​	hunter2_gazebo: The file is Gazebo simulation function package

​	steer_bot_hardware_gazebo: Gazebo simulation plug-in of Ackerman steering robot

​	steer_drive_controller:  Controller of Ackerman steering robot

​	（[steer_bot_hardware_gazebo](http://wiki.ros.org/steer_bot_hardware_gazebo?distro=indigo)and[steer_drive_controller](http://wiki.ros.org/steer_drive_controller?distro=indigo)are provided for official website) 

## 2.Environment

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

​	Download and install rqt-robot-steering plug-in, rqt_robot_steering is a ROS tool closely related to robot motion control, it can send the control command of robot linear motion and steering motion, and the robot motion can be easily controlled through the sliding bar

```
sudo apt-get install ros-melodic-rqt-robot-steering 
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

​		Confirm whether the dependency of the function package is installed

```
rosdep install --from-paths src --ignore-src -r -y 
```

​		Compile

```
catkin_make
```

### 	2.Run the star file of hunter2.0 model and visualize the model in Rviz

​		Enter the hunter_ws folder

```
cd hunter_ws
```

​		Declare the environment variable

```
source devel/setup.bash
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
source devel/setup.bash
```

​		Start the simulation environment of hunter2.0, slide the sliding bar of Robot Steering plug-in to control robot movement

```
roslaunch hunter2_gazebo hunter2_gazebo.launch
```

![说明文字](image/gazebo.png)



reference：https://github.com/CIR-KIT/steer_drive_ros

