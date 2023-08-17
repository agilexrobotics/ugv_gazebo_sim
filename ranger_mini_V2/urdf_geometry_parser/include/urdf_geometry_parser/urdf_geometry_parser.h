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

#ifndef URDF_GEOMETRY_PARSER_URDF_GEOMETRY_PARSER_H
#define URDF_GEOMETRY_PARSER_URDF_GEOMETRY_PARSER_H

#include <ros/ros.h>

#include <urdf/urdfdom_compatibility.h>
#include <urdf_parser/urdf_parser.h>

namespace urdf_geometry_parser {

class UrdfGeometryParser {

public:
  UrdfGeometryParser(ros::NodeHandle& root_nh, const std::string &base_link);

  /**
   * \brief Get transform vector between the joint and parent_link
   * \param [in] joint_name   Child joint_name from where to begin
   * \param [in] parent_link_name Name of link to find as parent of the joint
   * \param [out] transform_vector Distance vector between joint and parent_link [m]
   * \return true if the transform vector was found; false otherwise
   */
  bool getTransformVector(const std::string& joint_name,
                          const std::string& parent_link_name,
                          urdf::Vector3& transform_vector);

  /**
   * \brief Get distance between two joints from the URDF
   * \param first_joint_name Name of the first joint
   * \param second_joint_name Name of the second joint
   * \param distance Distance in meter between first and second joint
   * \return true if the distance was found; false otherwise
   */
  bool getDistanceBetweenJoints(const std::string& first_joint_name,
                                const std::string& second_joint_name,
                                double& distance);

  /**
   * \brief Get joint radius from the URDF
   * \param joint_name Name of the joint
   * \param distance Distance in meter between first and second joint
   * \return true if the radius was found; false otherwise
   */
  bool getJointRadius(const std::string& joint_name,
                                double& radius);

  /**
   * \brief Get joint steering limit from the URDF
   *        considering the upper and lower limit is the same
   * \param joint_name Name of the joint
   * \param steering_limit [rad]
   * \return true if the steering limit was found; false otherwise
   */
  bool getJointSteeringLimits(const std::string& joint_name,
                              double& steering_limit);
private:
  std::string base_link_;

  urdf::ModelInterfaceSharedPtr model_;
};

} // urdf_geometry_parser

#endif
