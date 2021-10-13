#ifndef GAZEBO_ROS_ACKERMAN_DRIVE_H_
#define GAZEBO_ROS_ACKERMAN_DRIVE_H_

#include <map>

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

// ROS
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Float64.h>

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

// Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>

namespace gazebo {

class Joint;
class Entity;

class GazeboRosAckermanDrive : public ModelPlugin {
public:
    GazeboRosAckermanDrive();
    ~GazeboRosAckermanDrive();
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

protected:
    virtual void UpdateChild();
    virtual void FiniChild();

private:
    void PublishOdometry(double step_time);
    void GetWheelVelocities();
    void ConvertCentralAngleToLeftRight(double angle,double r, double& left_angle, double& right_angle);
    void QueueThread();
    void CmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg);

    physics::WorldPtr world;
    physics::ModelPtr parent;
    event::ConnectionPtr update_connection_;
    std_msgs::Float64 last_motor_cmd[2];
    std::string left_front_joint_name_;
    std::string right_front_joint_name_;
    std::string left_rear_joint_name_;
    std::string right_rear_joint_name_;
    std::string left_hinge_joint_name_;
    std::string right_hinge_joint_name_;
    double wheel_separation_;
    double wheel_diameter_;
    double torque;
    double wheel_speed_[4];

    physics::JointPtr joints[4];

    // ROS STUFF
    ros::NodeHandle* rosnode_;
    ros::Publisher odometry_publisher_;
    ros::Subscriber cmd_vel_subscriber_;
    ros::Publisher motor_steer_fr_pub_;
    ros::Publisher motor_steer_fl_pub_;
    tf::TransformBroadcaster *transform_broadcaster_;
    nav_msgs::Odometry odom_;
    std::string tf_prefix_;
    bool broadcast_tf_;

    boost::mutex lock;

    std::string robot_namespace_;
    std::string command_topic_;
    std::string odometry_topic_;
    std::string odometry_frame_;
    std::string robot_base_frame_;

    static constexpr double max_steer_angle_central = 0.523598767; // ~= 30
    static constexpr double track_ = 0.172;   // m (left right wheel distance)
    static constexpr double wheelbase_ = 0.2; // m (front rear wheel distance)

    // Custom Callback Queue
    ros::CallbackQueue queue_;
    boost::thread callback_queue_thread_;

    double x_;
    double rot_;
    bool alive_;

    // Update Rate
    double update_rate_;
    double update_period_;
    common::Time last_update_time_;

    double covariance_x_;
    double covariance_y_;
    double covariance_yaw_;
};

}

#endif /* GAZEBO_ROS_ACKERMAN_DRIVE_H_ */
