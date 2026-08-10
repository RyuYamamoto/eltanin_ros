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

#include "eltanin_ros_common/directed_path_conversion.hpp"
#include "test/conversion_test_helpers.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <limits>
#include <vector>

namespace
{

using eltanin::Direction;
using eltanin_ros_common::to_directed_path_msg;
using eltanin_ros_common::to_direction;
using eltanin_ros_common::to_direction_msg;
using eltanin_ros_common::to_path;
using eltanin_ros_common::test::contains;
using eltanin_ros_common::test::is_one_line;
using eltanin_ros_common::test::names_the_package_once;

using DirectedPath = eltanin_msgs::msg::DirectedPath;

constexpr double NOT_A_NUMBER = std::numeric_limits<double>::quiet_NaN();

builtin_interfaces::msg::Time make_stamp(std::int32_t sec)
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = sec;
  return stamp;
}

eltanin::Path make_cusp_path()
{
  return eltanin::Path(
    std::vector<eltanin::Pose2D>{
      eltanin::Pose2D{Eigen::Vector2d{0.0, 0.0}, 0.0},
      eltanin::Pose2D{Eigen::Vector2d{1.5, -2.25}, 0.75},
      eltanin::Pose2D{Eigen::Vector2d{-3.0, 4.0}, -2.5},
      eltanin::Pose2D{Eigen::Vector2d{-3.0, 4.0}, 1.0}},
    std::vector<Direction>{Direction::Forward, Direction::Reverse, Direction::InPlace});
}

}  // namespace

TEST(DirectedPathRoundTripTest, KeepsEveryDirection)
{
  const eltanin::Path path = make_cusp_path();
  const auto msg = to_directed_path_msg(path, "map", make_stamp(7));
  ASSERT_TRUE(msg.ok()) << msg.error();
  EXPECT_EQ(msg.value().header.frame_id, "map");
  EXPECT_EQ(msg.value().header.stamp.sec, 7);
  ASSERT_EQ(msg.value().segment_directions.size(), path.size() - 1);

  const auto back = to_path(msg.value());
  ASSERT_TRUE(back.ok()) << back.error();
  ASSERT_EQ(back.value().size(), path.size());
  EXPECT_TRUE(back.value().has_directions());
  EXPECT_EQ(back.value().directions(), path.directions());
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    EXPECT_EQ(back.value().direction_of(i), path.direction_of(i)) << "segment " << i;
  }
  EXPECT_TRUE(back.value().has_reverse());
  EXPECT_TRUE(back.value().is_cusp(1));
}

TEST(DirectedPathRoundTripTest, KeepsPositionsExactlyAndYawWithinRounding)
{
  const eltanin::Path path = make_cusp_path();
  const auto msg = to_directed_path_msg(path, "map", make_stamp(0));
  ASSERT_TRUE(msg.ok());
  const auto back = to_path(msg.value());
  ASSERT_TRUE(back.ok());

  for (std::size_t i = 0; i < path.size(); ++i) {
    EXPECT_DOUBLE_EQ(back.value()[i].position.x(), path[i].position.x()) << "pose " << i;
    EXPECT_DOUBLE_EQ(back.value()[i].position.y(), path[i].position.y()) << "pose " << i;
    EXPECT_NEAR(back.value()[i].yaw, path[i].yaw, 1e-12) << "pose " << i;
  }
}

TEST(DirectedPathRoundTripTest, AnAllForwardPathCarriesNoDirectionArray)
{
  const eltanin::Path path{
    eltanin::Pose2D{Eigen::Vector2d{0.0, 0.0}, 0.0},
    eltanin::Pose2D{Eigen::Vector2d{1.0, 0.0}, 0.0}};
  ASSERT_FALSE(path.has_directions());

  const auto msg = to_directed_path_msg(path, "map", make_stamp(0));
  ASSERT_TRUE(msg.ok());
  EXPECT_TRUE(msg.value().segment_directions.empty());

  const auto back = to_path(msg.value());
  ASSERT_TRUE(back.ok());
  EXPECT_FALSE(back.value().has_directions());
  EXPECT_EQ(back.value().direction_of(0), Direction::Forward);
}

TEST(DirectedPathRoundTripTest, AnEmptyPathIsNotAnError)
{
  const auto msg = to_directed_path_msg(eltanin::Path{}, "map", make_stamp(0));
  ASSERT_TRUE(msg.ok());
  EXPECT_TRUE(msg.value().poses.empty());

  const auto back = to_path(msg.value());
  ASSERT_TRUE(back.ok());
  EXPECT_TRUE(back.value().empty());
}

TEST(DirectedPathConversionTest, EveryDirectionRoundTripsThroughItsWireValue)
{
  for (const Direction direction : {Direction::Forward, Direction::Reverse, Direction::InPlace}) {
    const auto back = to_direction(to_direction_msg(direction));
    ASSERT_TRUE(back.ok()) << back.error();
    EXPECT_EQ(back.value(), direction);
  }
  EXPECT_EQ(to_direction_msg(Direction::Forward), DirectedPath::DIRECTION_FORWARD);
  EXPECT_EQ(to_direction_msg(Direction::Reverse), DirectedPath::DIRECTION_REVERSE);
  EXPECT_EQ(to_direction_msg(Direction::InPlace), DirectedPath::DIRECTION_IN_PLACE);
}

TEST(DirectedPathConversionTest, AnUnknownDirectionValueIsRejected)
{
  const auto rejected = to_direction(9);
  ASSERT_FALSE(rejected.ok());
  EXPECT_TRUE(is_one_line(rejected.error()));
  EXPECT_TRUE(contains(rejected.error(), "9"));
}

TEST(DirectedPathConversionTest, AMisSizedDirectionArrayIsRejected)
{
  DirectedPath msg;
  msg.header.frame_id = "map";
  msg.poses.resize(4);
  for (auto & pose : msg.poses) {
    pose.orientation.w = 1.0;
  }
  msg.segment_directions = {DirectedPath::DIRECTION_FORWARD, DirectedPath::DIRECTION_REVERSE};

  const auto rejected = to_path(msg);
  ASSERT_FALSE(rejected.ok());
  EXPECT_TRUE(is_one_line(rejected.error()));
  EXPECT_TRUE(names_the_package_once(rejected.error()));
  EXPECT_TRUE(contains(rejected.error(), "segment_directions"));
}

TEST(DirectedPathConversionTest, AnUnknownDirectionInsideThePathIsRejected)
{
  DirectedPath msg;
  msg.header.frame_id = "map";
  msg.poses.resize(3);
  for (auto & pose : msg.poses) {
    pose.orientation.w = 1.0;
  }
  msg.segment_directions = {DirectedPath::DIRECTION_FORWARD, 42};

  const auto rejected = to_path(msg);
  ASSERT_FALSE(rejected.ok());
  EXPECT_TRUE(is_one_line(rejected.error()));
  EXPECT_TRUE(contains(rejected.error(), "42"));
}

TEST(DirectedPathConversionTest, ANonFinitePoseIsRejectedOnTheWayOut)
{
  const eltanin::Path path{
    eltanin::Pose2D{Eigen::Vector2d{0.0, 0.0}, 0.0},
    eltanin::Pose2D{Eigen::Vector2d{NOT_A_NUMBER, 0.0}, 0.0}};

  const auto rejected = to_directed_path_msg(path, "map", make_stamp(0));
  ASSERT_FALSE(rejected.ok());
  EXPECT_TRUE(is_one_line(rejected.error()));
  EXPECT_TRUE(names_the_package_once(rejected.error()));
  EXPECT_TRUE(contains(rejected.error(), "index 1"));
}
