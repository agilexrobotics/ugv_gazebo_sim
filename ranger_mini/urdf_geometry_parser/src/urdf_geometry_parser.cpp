/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Irstea
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Irstea nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <urdf_geometry_parser/urdf_geometry_parser.h>

#include <cmath>

namespace urdf_geometry_parser{

  /*
   * \brief Check if the link is modeled as a cylinder
   * \param link Link
   * \return true if the link is modeled as a Cylinder; false otherwise
   */
  static bool isCylinder(urdf::LinkConstSharedPtr& link)
  {
    if (!link)
    {
      ROS_ERROR("Link == NULL.");
      return false;
    }

    if (!link->collision)
    {
      ROS_ERROR_STREAM("Link " << link->name << " does not have collision description. Add collision description for link to urdf.");
      return false;
    }

    if (!link->collision->geometry)
    {
      ROS_ERROR_STREAM("Link " << link->name << " does not have collision geometry description. Add collision geometry description for link to urdf.");
      return false;
    }

    if (link->collision->geometry->type != urdf::Geometry::CYLINDER)
    {
      ROS_ERROR_STREAM("Link " << link->name << " does not have cylinder geometry");
      return false;
    }

    return true;
  }

  /*
   * \brief Get the wheel radius
   * \param [in]  wheel_link   Wheel link
   * \param [out] wheel_radius Wheel radius [m]
   * \return true if the wheel radius was found; false otherwise
   */
  static bool getWheelRadius(urdf::LinkConstSharedPtr wheel_link, double& wheel_radius)
  {
    if (!isCylinder(wheel_link))
    {
      ROS_ERROR_STREAM("Wheel link " << wheel_link->name << " is NOT modeled as a cylinder!");
      return false;
    }

    wheel_radius = (static_cast<urdf::Cylinder*>(wheel_link->collision->geometry.get()))->radius;
    return true;
  }


  UrdfGeometryParser::UrdfGeometryParser(ros::NodeHandle& root_nh, const std::string& base_link):
    base_link_(base_link)
  {
    // Parse robot description
    const std::string model_param_name = "robot_description";
    bool res = root_nh.hasParam(model_param_name);
    std::string robot_model_str="";
    if (!res || !root_nh.getParam(model_param_name,robot_model_str))
    {
      ROS_ERROR("Robot descripion couldn't be retrieved from param server.");
    }
    else
    {
      model_ = urdf::parseURDF(robot_model_str);
      if(!model_)
        ROS_ERROR_STREAM("Could not parse the urdf robot model "<<model_param_name);
    }
  }

  bool UrdfGeometryParser::getTransformVector(const std::string& joint_name, const std::string& parent_link_name
                                                , urdf::Vector3 &transform_vector)
  {
    if(model_)
    {
      urdf::JointConstSharedPtr joint(model_->getJoint(joint_name));
      if (!joint)
      {
        ROS_ERROR_STREAM(joint_name
                               << " couldn't be retrieved from model description");
        return false;
      }

      transform_vector = joint->parent_to_joint_origin_transform.position;
      while(joint->parent_link_name != parent_link_name)
      {
        urdf::LinkConstSharedPtr link_parent(model_->getLink(joint->parent_link_name));
        if (!link_parent || !link_parent->parent_joint)
        {
          ROS_ERROR_STREAM(joint->parent_link_name
                                 << " couldn't be retrieved from model description or his parent joint");
          return false;
        }
        joint = link_parent->parent_joint;
        transform_vector = transform_vector + joint->parent_to_joint_origin_transform.position;
      }
      return true;
    }
    else
      return false;
  }

  bool UrdfGeometryParser::getDistanceBetweenJoints(const std::string& first_joint_name,
                                                      const std::string& second_joint_name,
                                                      double& distance)
  {
    urdf::Vector3 first_transform;
    if(!getTransformVector(first_joint_name, base_link_, first_transform))
      return false;

    urdf::Vector3 second_transform;
    if(!getTransformVector(second_joint_name, base_link_, second_transform))
      return false;

    // Calculate the Euclidean distance using the Pythagorean formula
    distance = std::sqrt(std::pow(first_transform.x - second_transform.x, 2)
                         + std::pow(first_transform.y - second_transform.y, 2));
    return true;
  }

  bool UrdfGeometryParser::getJointRadius(const std::string& joint_name,
                                            double& radius)
  {
    if(model_)
    {
      urdf::JointConstSharedPtr joint(model_->getJoint(joint_name));
      // Get wheel radius
      if (!getWheelRadius(model_->getLink(joint->child_link_name), radius))
      {
        ROS_ERROR_STREAM("Couldn't retrieve " << joint_name << " wheel radius");
        return false;
      }
      return true;
    }
    else
      return false;
  }

  bool UrdfGeometryParser::getJointSteeringLimits(const std::string& joint_name,
                              double& steering_limit)
  {
    if(model_)
    {
      urdf::JointConstSharedPtr joint(model_->getJoint(joint_name));
      if(joint->type == urdf::Joint::REVOLUTE)
      {
        const double lower_steering_limit = fabs(joint->limits->lower);
        const double upper_steering_limit = fabs(joint->limits->upper);
        if(lower_steering_limit > upper_steering_limit)
          steering_limit = upper_steering_limit;
        else
          steering_limit = lower_steering_limit;
        ROS_DEBUG_STREAM("Joint "<<joint_name<<" steering limit is "<<steering_limit*180.0/M_PI<<" in degrees");
        return true;
      }
      ROS_ERROR_STREAM("Couldn't get joint "<<joint_name<<" steering limit, is it of type REVOLUTE ?");
    }
    return false;
  }
}
