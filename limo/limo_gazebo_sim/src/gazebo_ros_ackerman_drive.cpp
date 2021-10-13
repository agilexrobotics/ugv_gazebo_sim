#include <algorithm>
#include <assert.h>

#include <limo_gazebo/gazebo_ros_ackerman_drive.h>

#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <sdf/sdf.hh>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/Odometry.h>
#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>

namespace gazebo {

enum {
    RIGHT_FRONT=0,
    LEFT_FRONT=1,
    RIGHT_REAR=2,
    LEFT_REAR=3,
};

GazeboRosAckermanDrive::GazeboRosAckermanDrive() {}

// Destructor
GazeboRosAckermanDrive::~GazeboRosAckermanDrive() {
    delete rosnode_;
    delete transform_broadcaster_;
}

// Load the controller
void GazeboRosAckermanDrive::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
    this->parent = _parent;
    this->world = _parent->GetWorld();

    this->robot_namespace_ = "";
    if (!_sdf->HasElement("robotNamespace")) {
        ROS_INFO_NAMED("ackerman_drive", "GazeboRosAckermanDrive Plugin missing <robotNamespace>, defaults to \"%s\"",
                       this->robot_namespace_.c_str());
    }
    else {
        this->robot_namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";
    }

    this->broadcast_tf_ = false;
    if (!_sdf->HasElement("broadcastTF")) {
        if (!this->broadcast_tf_) {
            ROS_INFO_NAMED("ackerman_drive", "GazeboRosAckermanDrive Plugin (ns = %s) missing <broadcastTF>, defaults to false.",this->robot_namespace_.c_str());
        }
        else {
            ROS_INFO_NAMED("ackerman_drive", "GazeboRosAckermanDrive Plugin (ns = %s) missing <broadcastTF>, defaults to true.",this->robot_namespace_.c_str());
        }
    }
    else {
        this->broadcast_tf_ = _sdf->GetElement("broadcastTF")->Get<bool>();
    }

    this->left_front_joint_name_ = "left_front_joint";
    if (!_sdf->HasElement("leftFrontJoint")) {
        ROS_WARN_NAMED("ackerman_drive", "GazeboRosAckermanDrive Plugin (ns = %s) missing <leftFrontJoint>, defaults to \"%s\"",
                       this->robot_namespace_.c_str(), this->left_front_joint_name_.c_str());
    }
    else {
        this->left_front_joint_name_ = _sdf->GetElement("leftFrontJoint")->Get<std::string>();
    }

    this->right_front_joint_name_ = "right_front_joint";
    if (!_sdf->HasElement("rightFrontJoint")) {
        ROS_WARN_NAMED("ackerman_drive", "GazeboRosAckermanDrive Plugin (ns = %s) missing <rightFrontJoint>, defaults to \"%s\"",
                       this->robot_namespace_.c_str(), this->right_front_joint_name_.c_str());
    }
    else {
        this->right_front_joint_name_ = _sdf->GetElement("rightFrontJoint")->Get<std::string>();
    }

    this->left_rear_joint_name_ = "left_rear_joint";
    if (!_sdf->HasElement("leftRearJoint")) {
        ROS_WARN_NAMED("ackerman_drive", "GazeboRosAckermanDrive Plugin (ns = %s) missing <leftRearJoint>, defaults to \"%s\"",
                       this->robot_namespace_.c_str(), this->left_rear_joint_name_.c_str());
    }
    else {
        this->left_rear_joint_name_ = _sdf->GetElement("leftRearJoint")->Get<std::string>();
    }

    this->right_rear_joint_name_ = "right_rear_joint";
    if (!_sdf->HasElement("rightRearJoint")) {
        ROS_WARN_NAMED("ackerman_drive", "GazeboRosAckermanDrive Plugin (ns = %s) missing <rightRearJoint>, defaults to \"%s\"",
                       this->robot_namespace_.c_str(), this->right_rear_joint_name_.c_str());
    }
    else {
        this->right_rear_joint_name_ = _sdf->GetElement("rightRearJoint")->Get<std::string>();
    }

    this->left_hinge_joint_name_ = "left_hinge_joint";
    if (!_sdf->HasElement("leftHingeJoint")) {
        ROS_WARN_NAMED("ackerman_drive", "GazeboRosAckermanDrive Plugin (ns = %s) missing <leftHingeJoint>, defaults to \"%s\"",
                       this->robot_namespace_.c_str(), this->right_rear_joint_name_.c_str());
    }
    else {
        this->left_hinge_joint_name_ = _sdf->GetElement("leftHingeJoint")->Get<std::string>();
    }

    this->right_hinge_joint_name_ = "right_hinge_joint";
    if (!_sdf->HasElement("rightHingeJoint")) {
        ROS_WARN_NAMED("ackerman_drive", "GazeboRosAckermanDrive Plugin (ns = %s) missing <rightHingeJoint>, defaults to \"%s\"", this->robot_namespace_.c_str(), this->right_rear_joint_name_.c_str());
    }
    else {
        this->right_hinge_joint_name_ = _sdf->GetElement("rightHingeJoint")->Get<std::string>();
    }

    this->wheel_separation_ = 0.4;

    if (!_sdf->HasElement("wheelSeparation")) {
        ROS_WARN_NAMED("ackerman_drive", "GazeboRosAckermanDrive Plugin (ns = %s) missing <wheelSeparation>, defaults to value from robot_description: %f",
                       this->robot_namespace_.c_str(), this->wheel_separation_);
    }
    else {
        this->wheel_separation_ = _sdf->GetElement("wheelSeparation")->Get<double>();
    }

    this->wheel_diameter_ = 0.15;
    if (!_sdf->HasElement("wheelDiameter")) {
        ROS_WARN_NAMED("ackerman_drive", "GazeboRosAckermanDrive Plugin (ns = %s) missing <wheelDiameter>, defaults to %f", this->robot_namespace_.c_str(), this->wheel_diameter_);
    }
    else {
        this->wheel_diameter_ = _sdf->GetElement("wheelDiameter")->Get<double>();
    }

    this->torque = 5.0;
    if (!_sdf->HasElement("torque")) {
        ROS_WARN_NAMED("ackerman_drive", "GazeboRosAckermanDrive Plugin (ns = %s) missing <torque>, defaults to %f",
                       this->robot_namespace_.c_str(), this->torque);
    }
    else {
        this->torque = _sdf->GetElement("torque")->Get<double>();
    }

    this->command_topic_ = "cmd_vel";
    if (!_sdf->HasElement("commandTopic")) {
        ROS_WARN_NAMED("ackerman_drive", "GazeboRosAckermanDrive Plugin (ns = %s) missing <commandTopic>, defaults to \"%s\"",
                       this->robot_namespace_.c_str(), this->command_topic_.c_str());
    }
    else {
        this->command_topic_ = _sdf->GetElement("commandTopic")->Get<std::string>();
    }

    this->odometry_topic_ = "odom";
    if (!_sdf->HasElement("odometryTopic")) {
        ROS_WARN_NAMED("ackerman_drive", "GazeboRosAckermanDrive Plugin (ns = %s) missing <odometryTopic>, defaults to \"%s\"",
                       this->robot_namespace_.c_str(), this->odometry_topic_.c_str());
    }
    else {
        this->odometry_topic_ = _sdf->GetElement("odometryTopic")->Get<std::string>();
    }

    this->odometry_frame_ = "odom";
    if (!_sdf->HasElement("odometryFrame")) {
        ROS_WARN_NAMED("ackerman_drive", "GazeboRosAckermanDrive Plugin (ns = %s) missing <odometryFrame>, defaults to \"%s\"",
                       this->robot_namespace_.c_str(), this->odometry_frame_.c_str());
    }
    else {
        this->odometry_frame_ = _sdf->GetElement("odometryFrame")->Get<std::string>();
    }

    this->robot_base_frame_ = "base_footprint";
    if (!_sdf->HasElement("robotBaseFrame")) {
        ROS_WARN_NAMED("ackerman_drive", "GazeboRosAckermanDrive Plugin (ns = %s) missing <robotBaseFrame>, defaults to \"%s\"",
                       this->robot_namespace_.c_str(), this->robot_base_frame_.c_str());
    }
    else {
        this->robot_base_frame_ = _sdf->GetElement("robotBaseFrame")->Get<std::string>();
    }

    this->update_rate_ = 100.0;
    if (!_sdf->HasElement("updateRate")) {
        ROS_WARN_NAMED("ackerman_drive", "GazeboRosAckermanDrive Plugin (ns = %s) missing <updateRate>, defaults to %f",
                       this->robot_namespace_.c_str(), this->update_rate_);
    }
    else {
        this->update_rate_ = _sdf->GetElement("updateRate")->Get<double>();
    }

    this->covariance_x_ = 0.0001;
    if (!_sdf->HasElement("covariance_x")) {
        ROS_WARN_NAMED("ackerman_drive", "GazeboRosAckermanDrive Plugin (ns = %s) missing <covariance_x>, defaults to %f",
                       this->robot_namespace_.c_str(), covariance_x_);
    }
    else {
        covariance_x_ = _sdf->GetElement("covariance_x")->Get<double>();
    }

    this->covariance_y_ = 0.0001;
    if (!_sdf->HasElement("covariance_y")) {
        ROS_WARN_NAMED("ackerman_drive", "GazeboRosAckermanDrive Plugin (ns = %s) missing <covariance_y>, defaults to %f",
                       this->robot_namespace_.c_str(), covariance_y_);
    }
    else {
        covariance_y_ = _sdf->GetElement("covariance_y")->Get<double>();
    }

    this->covariance_yaw_ = 0.01;
    if (!_sdf->HasElement("covariance_yaw")) {
        ROS_WARN_NAMED("ackerman_drive", "GazeboRosAckermanDrive Plugin (ns = %s) missing <covariance_yaw>, defaults to %f",
                       this->robot_namespace_.c_str(), covariance_yaw_);
    }
    else {
        covariance_yaw_ = _sdf->GetElement("covariance_yaw")->Get<double>();
    }

    // Initialize update rate stuff
    if (this->update_rate_ > 0.0) {
        this->update_period_ = 1.0 / this->update_rate_;
    }
    else {
        this->update_period_ = 0.0;
    }

#if GAZEBO_MAJOR_VERSION >= 8
    last_update_time_ = this->world->SimTime();
#else
    last_update_time_ = this->world->GetSimTime();
#endif

    // Initialize velocity stuff
    wheel_speed_[RIGHT_FRONT] = 0;
    wheel_speed_[LEFT_FRONT] = 0;
    wheel_speed_[RIGHT_REAR] = 0;
    wheel_speed_[LEFT_REAR] = 0;

    x_ = 0;
    rot_ = 0;
    alive_ = true;

    joints[LEFT_FRONT] = this->parent->GetJoint(left_front_joint_name_);
    joints[RIGHT_FRONT] = this->parent->GetJoint(right_front_joint_name_);
    joints[LEFT_REAR] = this->parent->GetJoint(left_rear_joint_name_);
    joints[RIGHT_REAR] = this->parent->GetJoint(right_rear_joint_name_);

    if (!joints[LEFT_FRONT]) {
        char error[200];
        snprintf(error, 200, "GazeboRosAckermanDrive Plugin (ns = %s) couldn't get left front hinge joint named \"%s\"",
                 this->robot_namespace_.c_str(), this->left_front_joint_name_.c_str());
        gzthrow(error);
    }

    if (!joints[RIGHT_FRONT]) {
        char error[200];
        snprintf(error, 200, "GazeboRosAckermanDrive Plugin (ns = %s) couldn't get right front hinge joint named \"%s\"",
                 this->robot_namespace_.c_str(), this->right_front_joint_name_.c_str());
        gzthrow(error);
    }

    if (!joints[LEFT_REAR]) {
       char error[200];
       snprintf(error, 200, "GazeboRosAckermanDrive Plugin (ns = %s) couldn't get left rear hinge joint named \"%s\"",
                this->robot_namespace_.c_str(), this->left_rear_joint_name_.c_str());
       gzthrow(error);
   }

   if (!joints[RIGHT_REAR]) {
       char error[200];
       snprintf(error, 200, "GazeboRosAckermanDrive Plugin (ns = %s) couldn't get right rear hinge joint named \"%s\"",
                this->robot_namespace_.c_str(), this->right_rear_joint_name_.c_str());
       gzthrow(error);
   }

#if GAZEBO_MAJOR_VERSION > 2
    joints[LEFT_FRONT]->SetParam("fmax", 0, torque);
    joints[RIGHT_FRONT]->SetParam("fmax", 0, torque);
    joints[LEFT_REAR]->SetParam("fmax", 0, torque);
    joints[RIGHT_REAR]->SetParam("fmax", 0, torque);
#else
    joints[LEFT_FRONT]->SetMaxForce(0, torque);
    joints[RIGHT_FRONT]->SetMaxForce(0, torque);
    joints[LEFT_REAR]->SetMaxForce(0, torque);
    joints[RIGHT_REAR]->SetMaxForce(0, torque);
#endif

    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized()) {
        ROS_FATAL_STREAM_NAMED("ackerman_drive", "A ROS node for Gazebo has not been initialized, unable to load plugin. "
                               << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
        return;
    }

    rosnode_ = new ros::NodeHandle(this->robot_namespace_);

    ROS_INFO_NAMED("ackerman_drive", "Starting GazeboRosAckermanDrive Plugin (ns = %s)", this->robot_namespace_.c_str());

    tf_prefix_ = tf::getPrefixParam(*rosnode_);
    transform_broadcaster_ = new tf::TransformBroadcaster();

    // ROS: Subscribe to the velocity command topic (usually "cmd_vel")
    ros::SubscribeOptions so = ros::SubscribeOptions::create<geometry_msgs::Twist>(command_topic_, 1,
        boost::bind(&GazeboRosAckermanDrive::CmdVelCallback, this, _1), ros::VoidPtr(), &queue_);

    cmd_vel_subscriber_ = rosnode_->subscribe(so);
    std::string motor_steer_fr_topic_ = robot_namespace_ + "/limo_fr_steering_hinge_controller/command";
    std::string motor_steer_fl_topic_ = robot_namespace_ + "/limo_fl_steering_hinge_controller/command";
    motor_steer_fr_pub_ = rosnode_->advertise<std_msgs::Float64>(motor_steer_fr_topic_, 50);
    motor_steer_fl_pub_ = rosnode_->advertise<std_msgs::Float64>(motor_steer_fl_topic_, 50);
    odometry_publisher_ = rosnode_->advertise<nav_msgs::Odometry>(odometry_topic_, 1);
    last_motor_cmd[0].data=0;
    last_motor_cmd[1].data=0;
    // start custom queue for diff drive
    this->callback_queue_thread_ = boost::thread(boost::bind(&GazeboRosAckermanDrive::QueueThread, this));

    // listen to the update event (broadcast every simulation iteration)
    this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&GazeboRosAckermanDrive::UpdateChild, this));
}

// Update the controller
void GazeboRosAckermanDrive::UpdateChild() {
#if GAZEBO_MAJOR_VERSION >= 8
    common::Time current_time = this->world->SimTime();
#else
    common::Time current_time = this->world->GetSimTime();
#endif
    double seconds_since_last_update = (current_time - last_update_time_).Double();
    if (seconds_since_last_update > update_period_) {
        PublishOdometry(seconds_since_last_update);

        // Update robot in case new velocities have been requested
        GetWheelVelocities();
#if GAZEBO_MAJOR_VERSION > 2
        joints[LEFT_FRONT]->SetParam("vel", 0, wheel_speed_[LEFT_FRONT] / (wheel_diameter_ / 2.0));
        joints[RIGHT_FRONT]->SetParam("vel", 0, wheel_speed_[RIGHT_FRONT] / (wheel_diameter_ / 2.0));
        joints[LEFT_REAR]->SetParam("vel", 0, wheel_speed_[LEFT_REAR] / (wheel_diameter_ / 2.0));
        joints[RIGHT_REAR]->SetParam("vel", 0, wheel_speed_[RIGHT_REAR] / (wheel_diameter_ / 2.0));
#else
        joints[LEFT_FRONT]->SetVelocity(0, wheel_speed_[LEFT_FRONT] / (wheel_diameter_ / 2.0));
        joints[RIGHT_FRONT]->SetVelocity(0, wheel_speed_[RIGHT_FRONT] / (wheel_diameter_ / 2.0));
        joints[LEFT_REAR]->SetVelocity(0, wheel_speed_[LEFT_REAR] / (wheel_diameter_ / 2.0));
        joints[RIGHT_REAR]->SetVelocity(0, wheel_speed_[RIGHT_REAR] / (wheel_diameter_ / 2.0));
#endif
        last_update_time_+= common::Time(update_period_);
    }
}

// Finalize the controller
void GazeboRosAckermanDrive::FiniChild() {
    alive_ = false;
    queue_.clear();
    queue_.disable();
    rosnode_->shutdown();
    callback_queue_thread_.join();
}

void GazeboRosAckermanDrive::GetWheelVelocities() {
    boost::mutex::scoped_lock scoped_lock(lock);
    double vr = x_;
    double va = rot_;
    double steer_cmd;
    double left_side_velocity  = vr;
    double right_side_velocity = vr;
    double r;
    int sig = 1;

    r = fabs(vr / va);
    steer_cmd = std::atan(wheelbase_ / r);
    steer_cmd = va < 0 ? -1 * steer_cmd : steer_cmd;
    if (steer_cmd > max_steer_angle_central) {
        steer_cmd = max_steer_angle_central;
    }
    if (steer_cmd < -max_steer_angle_central) {
        steer_cmd = -max_steer_angle_central;
    }
    sig = steer_cmd < 0 ? -1 : 1;
    ConvertCentralAngleToLeftRight(steer_cmd, r,last_motor_cmd[1].data, last_motor_cmd[0].data);
    if (fabs(steer_cmd) > 0){
        left_side_velocity = vr - sig * vr / r * track_ * 0.2;
        right_side_velocity = vr + sig * vr / r * track_ * 0.2;
    }
    if(vr != 0.0) {
        wheel_speed_[RIGHT_FRONT] = right_side_velocity / fabs(std::cos(last_motor_cmd[0].data));
        wheel_speed_[RIGHT_REAR] = right_side_velocity;
        wheel_speed_[LEFT_FRONT] = left_side_velocity / fabs(std::cos(last_motor_cmd[1].data));
        wheel_speed_[LEFT_REAR] = left_side_velocity;
    }
    else {
        wheel_speed_[RIGHT_FRONT] = 0;
        wheel_speed_[RIGHT_REAR] = 0;
        wheel_speed_[LEFT_FRONT] = 0;
        wheel_speed_[LEFT_REAR] = 0;
    }
    motor_steer_fr_pub_.publish(last_motor_cmd[0]);
    motor_steer_fl_pub_.publish(last_motor_cmd[1]);
}

void GazeboRosAckermanDrive::ConvertCentralAngleToLeftRight(double angle,double r, double&left_angle, double& right_angle) {
    double inner = 0;
    double outer = 0;
    left_angle = 0.0;
    right_angle = 0.0;

    if (fabs(angle) > 0) {
        inner = fabs(std::atan2(wheelbase_, (r - track_ * 0.5)));
        outer = fabs(std::atan2(wheelbase_, (r + track_ * 0.5)));
        if (angle > 0) {
            left_angle  = inner;
            right_angle = outer;
        }
        else {
            left_angle  = -outer;
            right_angle = -inner;
        }
        left_angle = left_angle > max_steer_angle_central ? max_steer_angle_central:
                      (left_angle < -max_steer_angle_central? -max_steer_angle_central : left_angle);
        right_angle = right_angle > max_steer_angle_central ? max_steer_angle_central:
                      (right_angle < -max_steer_angle_central? -max_steer_angle_central : right_angle);
    }
}

void GazeboRosAckermanDrive::CmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg) {
    boost::mutex::scoped_lock scoped_lock(lock);
    x_ = cmd_msg->linear.x;
    rot_ = cmd_msg->angular.z;
}

void GazeboRosAckermanDrive::QueueThread() {
    static const double timeout = 0.01;

    while (alive_ && rosnode_->ok()) {
        queue_.callAvailable(ros::WallDuration(timeout));
    }
}

void GazeboRosAckermanDrive::PublishOdometry(double step_time) {
    ros::Time current_time = ros::Time::now();
    std::string odom_frame = tf::resolve(tf_prefix_, odometry_frame_);
    std::string base_footprint_frame = tf::resolve(tf_prefix_, robot_base_frame_);

#if GAZEBO_MAJOR_VERSION >= 8
    ignition::math::Pose3d pose = this->parent->WorldPose();
#else
    ignition::math::Pose3d pose = this->parent->GetWorldPose().Ign();
#endif

    tf::Quaternion qt(pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z(), pose.Rot().W());
    tf::Vector3 vt(pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z());

    tf::Transform base_footprint_to_odom(qt, vt);
    if (this->broadcast_tf_) {
        transform_broadcaster_->sendTransform(tf::StampedTransform(base_footprint_to_odom,
                                                                   current_time, odom_frame,
                                                                   base_footprint_frame));
    }

    // publish odom topic
    odom_.pose.pose.position.x = pose.Pos().X();
    odom_.pose.pose.position.y = pose.Pos().Y();

    odom_.pose.pose.orientation.x = pose.Rot().X();
    odom_.pose.pose.orientation.y = pose.Rot().Y();
    odom_.pose.pose.orientation.z = pose.Rot().Z();
    odom_.pose.pose.orientation.w = pose.Rot().W();
    odom_.pose.covariance[0] = this->covariance_x_;
    odom_.pose.covariance[7] = this->covariance_y_;
    odom_.pose.covariance[14] = 1000000000000.0;
    odom_.pose.covariance[21] = 1000000000000.0;
    odom_.pose.covariance[28] = 1000000000000.0;
    odom_.pose.covariance[35] = this->covariance_yaw_;

    // get velocity in /odom frame
    ignition::math::Vector3d linear;
#if GAZEBO_MAJOR_VERSION >= 8
    linear = this->parent->WorldLinearVel();
    odom_.twist.twist.angular.z = this->parent->WorldAngularVel().Z();
#else
    linear = this->parent->GetWorldLinearVel().Ign();
    odom_.twist.twist.angular.z = this->parent->GetWorldAngularVel().Ign().Z();
#endif

    // convert velocity to child_frame_id (aka base_footprint)
    float yaw = pose.Rot().Yaw();
    odom_.twist.twist.linear.x = cosf(yaw) * linear.X() + sinf(yaw) * linear.Y();
    odom_.twist.twist.linear.y = cosf(yaw) * linear.Y() - sinf(yaw) * linear.X();
    odom_.twist.covariance[0] = this->covariance_x_;
    odom_.twist.covariance[7] = this->covariance_y_;
    odom_.twist.covariance[14] = 1000000000000.0;
    odom_.twist.covariance[21] = 1000000000000.0;
    odom_.twist.covariance[28] = 1000000000000.0;
    odom_.twist.covariance[35] = this->covariance_yaw_;

    odom_.header.stamp = current_time;
    odom_.header.frame_id = odom_frame;
    odom_.child_frame_id = base_footprint_frame;

    odometry_publisher_.publish(odom_);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboRosAckermanDrive)
}
