cmake_minimum_required(VERSION 3.02)
project(bunker_gazebo_sim)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS    
  roslaunch
  roslint
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  gazebo_dev
  gazebo_msgs
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

