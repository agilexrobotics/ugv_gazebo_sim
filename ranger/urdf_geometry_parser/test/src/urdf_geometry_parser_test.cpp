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

#include <ros/ros.h>
#include <urdf_geometry_parser/urdf_geometry_parser.h>

#include <gtest/gtest.h>

class UrdfGeometryParserTest : public ::testing::Test{
public:
  UrdfGeometryParserTest() :
    ugp_(nh_, "base_link")
  {}

  ros::NodeHandle nh_;
  urdf_geometry_parser::UrdfGeometryParser ugp_;
};

TEST_F(UrdfGeometryParserTest, testTransformVector)
{
  urdf::Vector3 transform_vector;

  bool result = ugp_.getTransformVector("front_left_wheel", "base_link", transform_vector);
  EXPECT_TRUE(result);
  EXPECT_DOUBLE_EQ(transform_vector.x, ( 1.90 / 2));
  EXPECT_DOUBLE_EQ(transform_vector.y, ( 1.10 / 2));
  EXPECT_DOUBLE_EQ(transform_vector.z, (-0.66 / 2) + 0.28 - 0.19);
}

TEST_F(UrdfGeometryParserTest, testDistance)
{
  double track, wheel_base;

  bool result_track = ugp_.getDistanceBetweenJoints("front_left_wheel", "front_right_wheel", track);
  EXPECT_TRUE(result_track);
  EXPECT_DOUBLE_EQ(track, 1.1);

  bool result_wb = ugp_.getDistanceBetweenJoints("front_right_wheel", "rear_right_wheel", wheel_base);
  EXPECT_TRUE(result_wb);
  EXPECT_DOUBLE_EQ(wheel_base, 1.9);
}

TEST_F(UrdfGeometryParserTest, testRadius)
{
  double wheel_radius;
  bool result = ugp_.getJointRadius("front_left_wheel", wheel_radius);
  EXPECT_TRUE(result);
  EXPECT_DOUBLE_EQ(wheel_radius, 0.28);
}

TEST_F(UrdfGeometryParserTest, testJointSteeringLimits)
{
  double steering_limit;
  bool result = ugp_.getJointSteeringLimits("rear_left_steering_joint", steering_limit);
  EXPECT_TRUE(result);
  EXPECT_NEAR(steering_limit, M_PI/6, 0.001);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "UrdfGeometryParserTestNode");
  return RUN_ALL_TESTS();
}
