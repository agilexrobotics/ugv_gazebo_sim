/*
 * tracer_skid_steer.cpp
 *
 * Created on: Mar 25, 2020 22:54
 * Description:
 *
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#include "tracer_gazebo/tracer_skid_steer.hpp"

#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>

namespace wescore {
TracerSkidSteer::TracerSkidSteer(ros::NodeHandle *nh, std::string robot_name)
    : nh_(nh), robot_name_(robot_name) {
  motor_r_topic_ = robot_name_ + "/tracer_motor_r_controller/command";
  motor_l_topic_ = robot_name_ + "/tracer_motor_l_controller/command";
//  motor_rl_topic_ = robot_name_ + "/tracer_motor_rl_controller/command";
//  motor_rr_topic_ = robot_name_ + "/tracer_motor_rr_controller/command";
  cmd_topic_ = robot_name_ + "/cmd_vel";
}

void TracerSkidSteer::SetupSubscription() {
  // command subscriber
  cmd_sub_ = nh_->subscribe<geometry_msgs::Twist>(
      cmd_topic_, 5, &TracerSkidSteer::TwistCmdCallback, this);

  // motor command publisher
  motor_r_pub_ = nh_->advertise<std_msgs::Float64>(motor_r_topic_, 50);
  motor_l_pub_ = nh_->advertise<std_msgs::Float64>(motor_l_topic_, 50);
//  motor_rl_pub_ = nh_->advertise<std_msgs::Float64>(motor_rl_topic_, 50);
//  motor_rr_pub_ = nh_->advertise<std_msgs::Float64>(motor_rr_topic_, 50);
}

void TracerSkidSteer::TwistCmdCallback(
    const geometry_msgs::Twist::ConstPtr &msg) {
  std_msgs::Float64 motor_cmd[2];

  double driving_vel = msg->linear.x;
  double steering_vel = msg->angular.z;

  double left_side_velocity =
      (driving_vel - steering_vel * TRACER_WHEELBASE) / TRACER_WHEEL_RADIUS;
  double right_side_velocity =
      (driving_vel + steering_vel * TRACER_WHEELBASE) / TRACER_WHEEL_RADIUS;

  motor_cmd[0].data = right_side_velocity;
  motor_cmd[1].data = -left_side_velocity;
//  motor_cmd[2].data = -left_side_velocity;
//  motor_cmd[3].data = right_side_velocity;

  motor_r_pub_.publish(motor_cmd[0]);
  motor_l_pub_.publish(motor_cmd[1]);
//  motor_rl_pub_.publish(motor_cmd[2]);
//  motor_rr_pub_.publish(motor_cmd[3]);
}

}  // namespace wescore
