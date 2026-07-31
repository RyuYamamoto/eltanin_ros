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

#include "eltanin_ros_common/path_conversion.hpp"
#include "test/conversion_test_helpers.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <limits>
#include <vector>

namespace
{

using eltanin_ros_common::to_path;
using eltanin_ros_common::to_path_msg;
using eltanin_ros_common::test::contains;
using eltanin_ros_common::test::is_one_line;

constexpr double NOT_A_NUMBER = std::numeric_limits<double>::quiet_NaN();

builtin_interfaces::msg::Time make_stamp(std::int32_t sec)
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = sec;
  return stamp;
}

eltanin::Path make_path()
{
  return eltanin::Path{
    eltanin::Pose2D{Eigen::Vector2d{0.0, 0.0}, 0.0},
    eltanin::Pose2D{Eigen::Vector2d{1.5, -2.25}, 0.75},
    eltanin::Pose2D{Eigen::Vector2d{-3.0, 4.0}, -2.5}};
}

TEST(PathRoundTripTest, KeepsPositionsExactlyAndYawWithinRounding)
{
  const eltanin::Path original = make_path();
  const auto msg = to_path_msg(original, "map", make_stamp(3));
  ASSERT_TRUE(msg.ok()) << msg.error();
  const auto back = to_path(msg.value());
  ASSERT_TRUE(back.ok()) << back.error();

  ASSERT_EQ(back.value().size(), original.size());
  for (std::size_t i = 0; i < original.size(); ++i) {
    EXPECT_DOUBLE_EQ(back.value()[i].position.x(), original[i].position.x()) << "index " << i;
    EXPECT_DOUBLE_EQ(back.value()[i].position.y(), original[i].position.y()) << "index " << i;
    EXPECT_NEAR(back.value()[i].yaw, original[i].yaw, 1e-12) << "index " << i;
  }
}

TEST(PathRoundTripTest, AnEmptyPathIsNotAnError)
{
  const auto msg = to_path_msg(eltanin::Path{}, "map", make_stamp(0));
  ASSERT_TRUE(msg.ok()) << msg.error();
  EXPECT_TRUE(msg.value().poses.empty());
  EXPECT_EQ(msg.value().header.frame_id, "map");

  const auto back = to_path(msg.value());
  ASSERT_TRUE(back.ok()) << back.error();
  EXPECT_TRUE(back.value().empty());
}

TEST(ToPathMsgTest, RepeatsTheHeaderOnEveryPoseAndFlattensZ)
{
  const auto msg = to_path_msg(make_path(), "odom", make_stamp(9));
  ASSERT_TRUE(msg.ok()) << msg.error();
  ASSERT_EQ(msg.value().poses.size(), 3u);
  for (const auto & stamped : msg.value().poses) {
    EXPECT_EQ(stamped.header.frame_id, "odom");
    EXPECT_EQ(stamped.header.stamp.sec, 9);
    EXPECT_DOUBLE_EQ(stamped.pose.position.z, 0.0);
  }
}

TEST(ToPathMsgTest, RejectsANonFinitePoseAndNamesTheIndex)
{
  eltanin::Path path = make_path();
  path[1].yaw = NOT_A_NUMBER;
  const auto rejected = to_path_msg(path, "map", make_stamp(0));
  EXPECT_FALSE(rejected.ok());
  EXPECT_TRUE(contains(rejected.error(), "index 1"));
  EXPECT_TRUE(contains(rejected.error(), "finite"));
  EXPECT_TRUE(is_one_line(rejected.error()));

  eltanin::Path position = make_path();
  position[2].position.x() = std::numeric_limits<double>::infinity();
  const auto bad_position = to_path_msg(position, "map", make_stamp(0));
  EXPECT_FALSE(bad_position.ok());
  EXPECT_TRUE(contains(bad_position.error(), "index 2"));
}

TEST(ToPathTest, AcceptsAnEmptyPerPoseFrameId)
{
  auto msg = to_path_msg(make_path(), "map", make_stamp(0)).value();
  for (auto & stamped : msg.poses) {
    stamped.header.frame_id.clear();
  }
  EXPECT_TRUE(to_path(msg).ok());
}

TEST(ToPathTest, RejectsAPerPoseFrameIdThatDisagreesWithThePath)
{
  auto msg = to_path_msg(make_path(), "map", make_stamp(0)).value();
  msg.poses[2].header.frame_id = "odom";
  const auto rejected = to_path(msg);
  EXPECT_FALSE(rejected.ok());
  EXPECT_TRUE(contains(rejected.error(), "index 2"));
  EXPECT_TRUE(contains(rejected.error(), "odom"));
  EXPECT_TRUE(is_one_line(rejected.error()));
}

TEST(ToPathTest, RejectsABadQuaternionAndNamesTheIndex)
{
  auto msg = to_path_msg(make_path(), "map", make_stamp(0)).value();
  msg.poses[1].pose.orientation.x = 0.0;
  msg.poses[1].pose.orientation.y = 0.0;
  msg.poses[1].pose.orientation.z = 0.0;
  msg.poses[1].pose.orientation.w = 0.0;
  const auto rejected = to_path(msg);
  EXPECT_FALSE(rejected.ok());
  EXPECT_TRUE(contains(rejected.error(), "index 1"));
  EXPECT_TRUE(contains(rejected.error(), "norm"));
}

TEST(ToPathTest, RejectsANonFinitePosition)
{
  auto msg = to_path_msg(make_path(), "map", make_stamp(0)).value();
  msg.poses[0].pose.position.y = NOT_A_NUMBER;
  const auto rejected = to_path(msg);
  EXPECT_FALSE(rejected.ok());
  EXPECT_TRUE(contains(rejected.error(), "index 0"));
  EXPECT_TRUE(contains(rejected.error(), "finite"));
}

TEST(ToPathTest, IgnoresThePerPoseStamp)
{
  auto msg = to_path_msg(make_path(), "map", make_stamp(0)).value();
  msg.poses[1].header.stamp.sec = 99;
  const auto result = to_path(msg);
  ASSERT_TRUE(result.ok()) << result.error();
  EXPECT_EQ(result.value().size(), 3u);
}

}  // namespace
