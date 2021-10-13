/*
 * TRACER_skid_steer.hpp
 *
 * Created on: Mar 25, 2020 22:52
 * Description:
 *
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#ifndef TRACER_SKID_STEER_HPP
#define TRACER_SKID_STEER_HPP

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

#include <string>

namespace wescore {
class TracerSkidSteer {
 public:
  TracerSkidSteer(ros::NodeHandle *nh, std::string robot_name = "");

  void SetupSubscription();

 private:
  std::string robot_name_;
  std::string motor_r_topic_;
  std::string motor_l_topic_;
//  std::string motor_rl_topic_;
//  std::string motor_rr_topic_;
//  std::string motor_fr_topic_;
//  std::string motor_fl_topic_;
//  std::string motor_rr_topic_;
//  std::string motor_rl_topic_;

  std::string cmd_topic_;

  const double TRACER_WHEELBASE = 0.498;
  const double TRACER_WHEEL_RADIUS = 0.16459;

  ros::NodeHandle *nh_;

  ros::Publisher motor_r_pub_;
  ros::Publisher motor_l_pub_;
//  ros::Publisher motor_fr_pub_;
//  ros::Publisher motor_fl_pub_;
//  ros::Publisher motor_rr_pub_;
//  ros::Publisher motor_rl_pub_;
//  ros::Publisher motor_rl_pub_;
//  ros::Publisher motor_rr_pub_;

  ros::Subscriber cmd_sub_;

  void TwistCmdCallback(const geometry_msgs::Twist::ConstPtr &msg);
};
}  // namespace wescore

#endif /* TRACER_SKID_STEER_HPP */
