// Copyright 2026 RyuYamamoto.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License

#include "eltanin_ros_common/trajectory_conversion.hpp"
#include "test/conversion_test_helpers.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <numbers>

namespace
{

using eltanin_ros_common::to_path;
using eltanin_ros_common::test::contains;
using eltanin_ros_common::test::is_one_line;
using eltanin_ros_common::test::names_the_package_once;

constexpr double NOT_A_NUMBER = std::numeric_limits<double>::quiet_NaN();
constexpr double INFINITY_VALUE = std::numeric_limits<double>::infinity();

eltanin_msgs::msg::TrajectoryPoint2D make_point(double x, double y, double theta)
{
  eltanin_msgs::msg::TrajectoryPoint2D point;
  point.x = x;
  point.y = y;
  point.theta = theta;
  point.linear_velocity = 0.4;
  point.angular_velocity = -0.2;
  point.time_from_start.sec = 1;
  return point;
}

eltanin_msgs::msg::Trajectory2D make_trajectory()
{
  eltanin_msgs::msg::Trajectory2D msg;
  msg.header.frame_id = "map";
  msg.points.push_back(make_point(0.0, 0.0, 0.0));
  msg.points.push_back(make_point(1.5, -2.25, 0.75));
  msg.points.push_back(make_point(-3.0, 4.0, -2.5));
  return msg;
}

TEST(TrajectoryConversionTest, KeepsThePositionsAndTheYawExactly)
{
  const eltanin_msgs::msg::Trajectory2D msg = make_trajectory();
  const auto path = to_path(msg);
  ASSERT_TRUE(path.ok()) << path.error();

  ASSERT_EQ(path.value().size(), msg.points.size());
  for (std::size_t i = 0; i < msg.points.size(); ++i) {
    EXPECT_DOUBLE_EQ(path.value()[i].position.x(), msg.points[i].x) << "index " << i;
    EXPECT_DOUBLE_EQ(path.value()[i].position.y(), msg.points[i].y) << "index " << i;
    EXPECT_DOUBLE_EQ(path.value()[i].yaw, msg.points[i].theta) << "index " << i;
  }
}

TEST(TrajectoryConversionTest, AnEmptyTrajectoryIsNotAnError)
{
  eltanin_msgs::msg::Trajectory2D msg;
  msg.header.frame_id = "map";
  const auto path = to_path(msg);
  ASSERT_TRUE(path.ok()) << path.error();
  EXPECT_TRUE(path.value().empty());
}

TEST(TrajectoryConversionTest, NormalizesThetaIntoTheHalfOpenTurn)
{
  eltanin_msgs::msg::Trajectory2D msg;
  msg.header.frame_id = "map";
  msg.points.push_back(make_point(0.0, 0.0, 3.0 * std::numbers::pi));
  msg.points.push_back(make_point(1.0, 0.0, -5.0 * std::numbers::pi));

  const auto path = to_path(msg);
  ASSERT_TRUE(path.ok()) << path.error();
  EXPECT_NEAR(path.value()[0].yaw, std::numbers::pi, 1e-12);
  EXPECT_NEAR(path.value()[1].yaw, std::numbers::pi, 1e-12);
}

TEST(TrajectoryConversionTest, RejectsANonFinitePointAndNamesItsIndex)
{
  eltanin_msgs::msg::Trajectory2D msg = make_trajectory();
  msg.points[1].y = NOT_A_NUMBER;

  const auto path = to_path(msg);
  ASSERT_FALSE(path.ok());
  EXPECT_TRUE(is_one_line(path.error()));
  EXPECT_TRUE(names_the_package_once(path.error()));
  EXPECT_TRUE(contains(path.error(), "at index 1"));
  EXPECT_TRUE(contains(path.error(), "frame_id='map'"));
  EXPECT_TRUE(contains(path.error(), "3 points"));
  EXPECT_TRUE(contains(path.error(), "finite"));
}

TEST(TrajectoryConversionTest, RejectsANonFiniteThetaToo)
{
  eltanin_msgs::msg::Trajectory2D msg = make_trajectory();
  msg.points[0].theta = INFINITY_VALUE;

  const auto path = to_path(msg);
  ASSERT_FALSE(path.ok());
  EXPECT_TRUE(contains(path.error(), "at index 0"));
}

TEST(TrajectoryConversionTest, TheVelocityAnnotationDoesNotReachThePath)
{
  eltanin_msgs::msg::Trajectory2D msg = make_trajectory();
  const auto with_velocity = to_path(msg);
  ASSERT_TRUE(with_velocity.ok()) << with_velocity.error();

  for (eltanin_msgs::msg::TrajectoryPoint2D & point : msg.points) {
    point.linear_velocity = 99.0;
    point.angular_velocity = -99.0;
    point.time_from_start.sec = 1234;
  }
  const auto without_velocity = to_path(msg);
  ASSERT_TRUE(without_velocity.ok()) << without_velocity.error();

  ASSERT_EQ(with_velocity.value().size(), without_velocity.value().size());
  for (std::size_t i = 0; i < with_velocity.value().size(); ++i) {
    EXPECT_EQ(with_velocity.value()[i].position, without_velocity.value()[i].position);
    EXPECT_EQ(with_velocity.value()[i].yaw, without_velocity.value()[i].yaw);
  }
}

}  // namespace
