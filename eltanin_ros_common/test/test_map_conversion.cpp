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

#include "eltanin_ros_common/geometry_conversion.hpp"
#include "eltanin_ros_common/map_conversion.hpp"
#include "test/conversion_test_helpers.hpp"

#include <eltanin/map/cost_values.hpp>
#include <tf2/LinearMath/Quaternion.hpp>

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

namespace
{

using eltanin::map::CellRect;
using eltanin::map::FREE_SPACE;
using eltanin::map::INSCRIBED_INFLATED_OBSTACLE;
using eltanin::map::LETHAL_OBSTACLE;
using eltanin::map::NO_INFORMATION;
using eltanin_ros_common::apply_costmap_update;
using eltanin_ros_common::MapLimits;
using eltanin_ros_common::OccupancyThresholds;
using eltanin_ros_common::to_costmap;
using eltanin_ros_common::to_costmap_msg;
using eltanin_ros_common::to_costmap_update_msg;
using eltanin_ros_common::to_occupancy_grid;
using eltanin_ros_common::test::contains;
using eltanin_ros_common::test::is_one_line;
using eltanin_ros_common::test::names_the_package_once;

constexpr double RESOLUTION = 0.05;

/// nav_msgs/MapMetaData::resolution is float32, so the static map path only ever sees this value.
constexpr double GRID_RESOLUTION = static_cast<double>(static_cast<float>(RESOLUTION));
constexpr double NOT_A_NUMBER = std::numeric_limits<double>::quiet_NaN();

builtin_interfaces::msg::Time make_stamp(std::int32_t sec, std::uint32_t nanosec)
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = sec;
  stamp.nanosec = nanosec;
  return stamp;
}

nav_msgs::msg::OccupancyGrid make_grid(
  std::uint32_t width, std::uint32_t height, const std::vector<std::int8_t> & data)
{
  nav_msgs::msg::OccupancyGrid msg;
  msg.header.frame_id = "map";
  msg.info.resolution = RESOLUTION;
  msg.info.width = width;
  msg.info.height = height;
  msg.info.origin.position.x = 1.0;
  msg.info.origin.position.y = 2.0;
  msg.data = data;
  return msg;
}

eltanin_msgs::msg::Costmap make_costmap_msg(
  std::uint32_t width, std::uint32_t height, const std::vector<std::uint8_t> & data)
{
  eltanin_msgs::msg::Costmap msg;
  msg.header.frame_id = "odom";
  msg.info.resolution = RESOLUTION;
  msg.info.width = width;
  msg.info.height = height;
  msg.info.origin_x = -1.5;
  msg.info.origin_y = 0.5;
  msg.data = data;
  return msg;
}

eltanin::map::Costmap make_costmap(
  int width, int height, const std::vector<std::uint8_t> & data, double resolution = RESOLUTION)
{
  eltanin::map::Costmap costmap(
    eltanin::map::MapGeometry(width, height, resolution, Eigen::Vector2d{-1.5, 0.5}));
  costmap.data() = data;
  return costmap;
}

TEST(ToCostmapFromGridTest, AppliesTheStaticMapThresholds)
{
  const auto result = to_costmap(make_grid(2, 2, {0, 100, -1, 40}), OccupancyThresholds{});
  ASSERT_TRUE(result.ok()) << result.error();
  EXPECT_EQ(result.value().get(0, 0), FREE_SPACE);
  EXPECT_EQ(result.value().get(1, 0), LETHAL_OBSTACLE);
  EXPECT_EQ(result.value().get(0, 1), NO_INFORMATION);
  EXPECT_EQ(result.value().get(1, 1), NO_INFORMATION);
}

TEST(ToCostmapFromGridTest, CopiesTheGeometry)
{
  const auto result = to_costmap(make_grid(3, 2, {0, 0, 0, 0, 0, 0}), OccupancyThresholds{});
  ASSERT_TRUE(result.ok()) << result.error();
  EXPECT_EQ(result.value().size_x(), 3);
  EXPECT_EQ(result.value().size_y(), 2);
  EXPECT_DOUBLE_EQ(result.value().geometry().resolution(), GRID_RESOLUTION);
  EXPECT_DOUBLE_EQ(result.value().geometry().origin().x(), 1.0);
  EXPECT_DOUBLE_EQ(result.value().geometry().origin().y(), 2.0);
}

TEST(ToCostmapFromGridTest, KeepsTheRowOrderSoDataZeroIsTheLowerLeftCell)
{
  const auto result = to_costmap(make_grid(3, 2, {0, 100, 0, 0, 0, 100}), OccupancyThresholds{});
  ASSERT_TRUE(result.ok()) << result.error();
  EXPECT_EQ(result.value().get(0, 0), FREE_SPACE);
  EXPECT_EQ(result.value().get(1, 0), LETHAL_OBSTACLE);
  EXPECT_EQ(result.value().get(2, 1), LETHAL_OBSTACLE);

  const Eigen::Vector2d lower_left = result.value().geometry().map_to_world(0, 0);
  EXPECT_DOUBLE_EQ(lower_left.x(), 1.0 + 0.5 * GRID_RESOLUTION);
  EXPECT_DOUBLE_EQ(lower_left.y(), 2.0 + 0.5 * GRID_RESOLUTION);
}

TEST(ToCostmapFromGridTest, RejectsANonPositiveOrNonFiniteResolution)
{
  auto zero = make_grid(2, 2, {0, 0, 0, 0});
  zero.info.resolution = 0.0;
  const auto rejected = to_costmap(zero, OccupancyThresholds{});
  EXPECT_FALSE(rejected.ok());
  EXPECT_TRUE(contains(rejected.error(), "resolution"));
  EXPECT_TRUE(contains(rejected.error(), "frame_id='map'"));
  EXPECT_TRUE(contains(rejected.error(), "2x2"));
  EXPECT_TRUE(is_one_line(rejected.error()));

  auto negative = make_grid(2, 2, {0, 0, 0, 0});
  negative.info.resolution = -RESOLUTION;
  EXPECT_FALSE(to_costmap(negative, OccupancyThresholds{}).ok());

  auto not_a_number = make_grid(2, 2, {0, 0, 0, 0});
  not_a_number.info.resolution = NOT_A_NUMBER;
  EXPECT_FALSE(to_costmap(not_a_number, OccupancyThresholds{}).ok());
}

TEST(ToCostmapFromGridTest, RejectsAMapOfSizeZero)
{
  const auto no_width = to_costmap(make_grid(0, 2, {}), OccupancyThresholds{});
  EXPECT_FALSE(no_width.ok());
  EXPECT_TRUE(contains(no_width.error(), "width and height"));
  EXPECT_FALSE(to_costmap(make_grid(2, 0, {}), OccupancyThresholds{}).ok());
  EXPECT_FALSE(to_costmap(make_grid(0, 0, {}), OccupancyThresholds{}).ok());
}

TEST(ToCostmapFromGridTest, RejectsMoreCellsThanTheLimitAllows)
{
  auto huge = make_grid(4000, 4000, {});
  huge.data.clear();
  const auto rejected = to_costmap(huge, OccupancyThresholds{}, MapLimits{1000});
  EXPECT_FALSE(rejected.ok());
  EXPECT_TRUE(contains(rejected.error(), "max_cells"));
  EXPECT_TRUE(contains(rejected.error(), "1000"));
}

TEST(ToCostmapFromGridTest, RejectsADataLengthThatDisagreesWithTheSize)
{
  const auto rejected = to_costmap(make_grid(2, 2, {0, 0, 0}), OccupancyThresholds{});
  EXPECT_FALSE(rejected.ok());
  EXPECT_TRUE(contains(rejected.error(), "data has 3 entries"));
}

TEST(ToCostmapFromGridTest, RejectsANonFiniteOrigin)
{
  auto msg = make_grid(2, 2, {0, 0, 0, 0});
  msg.info.origin.position.x = NOT_A_NUMBER;
  const auto rejected = to_costmap(msg, OccupancyThresholds{});
  EXPECT_FALSE(rejected.ok());
  EXPECT_TRUE(contains(rejected.error(), "origin"));
  EXPECT_TRUE(contains(rejected.error(), "finite"));
}

TEST(ToCostmapFromGridTest, RejectsAZeroQuaternionOrigin)
{
  auto msg = make_grid(2, 2, {0, 0, 0, 0});
  msg.info.origin.orientation.x = 0.0;
  msg.info.origin.orientation.y = 0.0;
  msg.info.origin.orientation.z = 0.0;
  msg.info.origin.orientation.w = 0.0;
  const auto rejected = to_costmap(msg, OccupancyThresholds{});
  EXPECT_FALSE(rejected.ok());
  EXPECT_TRUE(contains(rejected.error(), "norm"));
}

TEST(ToCostmapFromGridTest, RejectsARollOnlyOriginThatWouldReadAsZeroYaw)
{
  auto msg = make_grid(2, 2, {0, 0, 0, 0});
  tf2::Quaternion rolled;
  rolled.setRPY(std::numbers::pi, 0.0, 0.0);
  msg.info.origin.orientation.x = rolled.x();
  msg.info.origin.orientation.y = rolled.y();
  msg.info.origin.orientation.z = rolled.z();
  msg.info.origin.orientation.w = rolled.w();
  const auto rejected = to_costmap(msg, OccupancyThresholds{});
  EXPECT_FALSE(rejected.ok());
  EXPECT_TRUE(contains(rejected.error(), "roll"));
}

TEST(ToCostmapFromGridTest, RejectsAPitchOnlyOrigin)
{
  auto msg = make_grid(2, 2, {0, 0, 0, 0});
  tf2::Quaternion pitched;
  pitched.setRPY(0.0, 0.2, 0.0);
  msg.info.origin.orientation.x = pitched.x();
  msg.info.origin.orientation.y = pitched.y();
  msg.info.origin.orientation.z = pitched.z();
  msg.info.origin.orientation.w = pitched.w();
  const auto rejected = to_costmap(msg, OccupancyThresholds{});
  EXPECT_FALSE(rejected.ok());
  EXPECT_TRUE(contains(rejected.error(), "pitch"));
}

TEST(ToCostmapFromGridTest, RejectsARotatedOrigin)
{
  auto msg = make_grid(2, 2, {0, 0, 0, 0});
  msg.info.origin.orientation = eltanin_ros_common::to_quaternion(0.1);
  const auto rejected = to_costmap(msg, OccupancyThresholds{});
  EXPECT_FALSE(rejected.ok());
  EXPECT_TRUE(contains(rejected.error(), "yaw"));
  EXPECT_TRUE(contains(rejected.error(), "tolerance"));
  EXPECT_TRUE(is_one_line(rejected.error()));
}

TEST(ToCostmapFromGridTest, ReportsANestedViolationAsOneSentence)
{
  auto zero_quaternion = make_grid(2, 2, {0, 0, 0, 0});
  zero_quaternion.info.origin.orientation.w = 0.0;
  const auto quaternion = to_costmap(zero_quaternion, OccupancyThresholds{});
  ASSERT_FALSE(quaternion.ok());
  EXPECT_TRUE(names_the_package_once(quaternion.error()));
  EXPECT_TRUE(contains(quaternion.error(), "origin quaternion"));

  const auto thresholds = to_costmap(make_grid(2, 2, {0, 0, 0, 0}), OccupancyThresholds{25, 65});
  ASSERT_FALSE(thresholds.ok());
  EXPECT_TRUE(names_the_package_once(thresholds.error()));
}

TEST(ToCostmapFromGridTest, AcceptsAnOriginRotationInsideTheTolerance)
{
  auto msg = make_grid(2, 2, {0, 0, 0, 0});
  msg.info.origin.orientation = eltanin_ros_common::to_quaternion(1e-9);
  EXPECT_TRUE(to_costmap(msg, OccupancyThresholds{}).ok());
}

TEST(ToCostmapFromGridTest, RejectsInvalidThresholds)
{
  const auto rejected = to_costmap(make_grid(2, 2, {0, 0, 0, 0}), OccupancyThresholds{25, 65});
  EXPECT_FALSE(rejected.ok());
  EXPECT_TRUE(contains(rejected.error(), "free_threshold"));
}

template <class Message, class = void>
struct HasOriginPose : std::false_type
{
};

template <class Message>
struct HasOriginPose<Message, std::void_t<decltype(std::declval<Message>().info.origin)>>
: std::true_type
{
};

TEST(ToCostmapFromMessageTest, HasNoOriginPoseToValidate)
{
  static_assert(HasOriginPose<nav_msgs::msg::OccupancyGrid>::value);
  static_assert(!HasOriginPose<eltanin_msgs::msg::Costmap>::value);
}

TEST(ToCostmapFromMessageTest, CopiesTheRawValuesAndTheGeometry)
{
  const std::vector<std::uint8_t> raw{
    FREE_SPACE, 1, INSCRIBED_INFLATED_OBSTACLE, LETHAL_OBSTACLE, NO_INFORMATION, 200};
  const auto result = to_costmap(make_costmap_msg(3, 2, raw));
  ASSERT_TRUE(result.ok()) << result.error();
  EXPECT_EQ(result.value().data(), raw);
  EXPECT_EQ(result.value().size_x(), 3);
  EXPECT_EQ(result.value().size_y(), 2);
  EXPECT_DOUBLE_EQ(result.value().geometry().origin().x(), -1.5);
  EXPECT_DOUBLE_EQ(result.value().geometry().origin().y(), 0.5);
}

TEST(ToCostmapFromMessageTest, RunsTheSameGridChecks)
{
  auto resolution = make_costmap_msg(2, 2, {0, 0, 0, 0});
  resolution.info.resolution = 0.0;
  EXPECT_FALSE(to_costmap(resolution).ok());

  EXPECT_FALSE(to_costmap(make_costmap_msg(0, 2, {})).ok());
  EXPECT_FALSE(to_costmap(make_costmap_msg(2, 2, {0, 0, 0})).ok());
  EXPECT_FALSE(to_costmap(make_costmap_msg(2, 2, {0, 0, 0, 0}), MapLimits{3}).ok());

  auto origin = make_costmap_msg(2, 2, {0, 0, 0, 0});
  origin.info.origin_y = NOT_A_NUMBER;
  EXPECT_FALSE(to_costmap(origin).ok());
}

TEST(CostmapRoundTripTest, LosesNothingThroughTheInternalTopic)
{
  std::vector<std::uint8_t> raw(6);
  for (std::size_t i = 0; i < raw.size(); ++i) {
    raw[i] = static_cast<std::uint8_t>(i * 50);
  }
  raw.back() = NO_INFORMATION;
  const eltanin::map::Costmap original = make_costmap(3, 2, raw);

  const auto msg = to_costmap_msg(original, "odom", make_stamp(7, 8));
  ASSERT_TRUE(msg.ok()) << msg.error();
  const auto back = to_costmap(msg.value());
  ASSERT_TRUE(back.ok()) << back.error();

  EXPECT_EQ(back.value().geometry(), original.geometry());
  EXPECT_EQ(back.value().data(), original.data());
}

TEST(CostmapRoundTripTest, KeepsEveryReservedValueDistinct)
{
  const std::vector<std::uint8_t> raw{
    FREE_SPACE, 1, 252, INSCRIBED_INFLATED_OBSTACLE, LETHAL_OBSTACLE, NO_INFORMATION};
  const auto msg = to_costmap_msg(make_costmap(6, 1, raw), "odom", make_stamp(0, 0));
  ASSERT_TRUE(msg.ok()) << msg.error();
  const auto back = to_costmap(msg.value());
  ASSERT_TRUE(back.ok()) << back.error();
  EXPECT_EQ(back.value().data(), raw);
}

TEST(ToCostmapMsgTest, TakesFrameIdAndStampFromItsArguments)
{
  const auto msg = to_costmap_msg(make_costmap(2, 1, {0, 0}), "local_map", make_stamp(12, 34));
  ASSERT_TRUE(msg.ok()) << msg.error();
  EXPECT_EQ(msg.value().header.frame_id, "local_map");
  EXPECT_EQ(msg.value().header.stamp.sec, 12);
  EXPECT_EQ(msg.value().header.stamp.nanosec, 34u);
}

TEST(ToCostmapMsgTest, RejectsADegenerateCostmapInsteadOfPublishingIt)
{
  const eltanin::map::Costmap empty;
  const auto rejected = to_costmap_msg(empty, "map", make_stamp(0, 0));
  EXPECT_FALSE(rejected.ok());
  EXPECT_TRUE(contains(rejected.error(), "resolution"));
  EXPECT_TRUE(is_one_line(rejected.error()));

  // MapGeometry now refuses zero dimensions, so no conversion can ever be handed such a map.
  EXPECT_THROW(make_costmap(0, 0, {}), std::invalid_argument);

  eltanin::map::Costmap mismatched = make_costmap(2, 2, {0, 0, 0, 0});
  mismatched.data().pop_back();
  const auto size_mismatch = to_costmap_msg(mismatched, "map", make_stamp(0, 0));
  EXPECT_FALSE(size_mismatch.ok());
  EXPECT_TRUE(contains(size_mismatch.error(), "cell vector"));
}

TEST(ToOccupancyGridTest, MapsTheCostRangeForVisualization)
{
  const std::vector<std::uint8_t> raw{
    FREE_SPACE, 1, 252, INSCRIBED_INFLATED_OBSTACLE, LETHAL_OBSTACLE, NO_INFORMATION};
  const auto msg = to_occupancy_grid(make_costmap(6, 1, raw), "map", make_stamp(1, 2));
  ASSERT_TRUE(msg.ok()) << msg.error();
  const std::vector<std::int8_t> expected{0, 1, 98, 99, 100, -1};
  EXPECT_EQ(msg.value().data, expected);
  EXPECT_EQ(msg.value().header.frame_id, "map");
  EXPECT_EQ(msg.value().header.stamp.sec, 1);
}

TEST(ToOccupancyGridTest, PublishesAnUnrotatedOrigin)
{
  const auto msg = to_occupancy_grid(make_costmap(2, 1, {0, 0}), "map", make_stamp(0, 0));
  ASSERT_TRUE(msg.ok()) << msg.error();
  EXPECT_DOUBLE_EQ(msg.value().info.origin.position.x, -1.5);
  EXPECT_DOUBLE_EQ(msg.value().info.origin.position.y, 0.5);
  EXPECT_DOUBLE_EQ(msg.value().info.origin.position.z, 0.0);
  EXPECT_DOUBLE_EQ(msg.value().info.origin.orientation.w, 1.0);
  EXPECT_DOUBLE_EQ(msg.value().info.origin.orientation.z, 0.0);
  EXPECT_EQ(msg.value().info.width, 2u);
  EXPECT_EQ(msg.value().info.height, 1u);
  EXPECT_FLOAT_EQ(msg.value().info.resolution, static_cast<float>(RESOLUTION));
}

TEST(ToOccupancyGridTest, NarrowsResolutionToFloat32BecauseNavMsgsCannotHoldMore)
{
  const auto msg = to_occupancy_grid(make_costmap(2, 1, {0, 0}, 0.1), "map", make_stamp(0, 0));
  ASSERT_TRUE(msg.ok()) << msg.error();
  EXPECT_NE(static_cast<double>(msg.value().info.resolution), 0.1);
  EXPECT_NEAR(static_cast<double>(msg.value().info.resolution), 0.1, 1e-7);
}

TEST(ToCostmapMsgTest, KeepsResolutionExactlyBecauseTheInternalTopicIsFloat64)
{
  const auto msg = to_costmap_msg(make_costmap(2, 1, {0, 0}, 0.1), "map", make_stamp(0, 0));
  ASSERT_TRUE(msg.ok()) << msg.error();
  EXPECT_DOUBLE_EQ(msg.value().info.resolution, 0.1);
}

TEST(ToOccupancyGridTest, RejectsTheSameDegenerateMapsAsTheInternalTopic)
{
  EXPECT_FALSE(to_occupancy_grid(eltanin::map::Costmap{}, "map", make_stamp(0, 0)).ok());
  // MapGeometry now refuses zero dimensions, so no conversion can ever be handed such a map.
  EXPECT_THROW(make_costmap(0, 0, {}), std::invalid_argument);
}

TEST(ToOccupancyGridTest, DoesNotSurviveARoundTripThroughTheStaticMapThresholds)
{
  const std::vector<std::uint8_t> raw{1, 3, 200};
  const auto msg = to_occupancy_grid(make_costmap(3, 1, raw), "map", make_stamp(0, 0));
  ASSERT_TRUE(msg.ok()) << msg.error();
  const auto back = to_costmap(msg.value(), OccupancyThresholds{});
  ASSERT_TRUE(back.ok()) << back.error();
  EXPECT_NE(back.value().data(), raw);
}

/// A 4x3 map of distinct cells, so a transposed or shifted patch cannot pass by accident.
eltanin::map::Costmap make_numbered_costmap()
{
  std::vector<std::uint8_t> data(12);
  for (std::size_t at = 0; at < data.size(); ++at) {
    data[at] = static_cast<std::uint8_t>(at + 1);
  }
  return make_costmap(4, 3, data);
}

/// The independent definition of the patch: read the same rectangle back through get().
std::vector<std::uint8_t> cut_out(const eltanin::map::Costmap & costmap, const CellRect & rect)
{
  std::vector<std::uint8_t> expected;
  for (int my = rect.min_y; my <= rect.max_y; ++my) {
    for (int mx = rect.min_x; mx <= rect.max_x; ++mx) {
      expected.push_back(costmap.get(mx, my).value());
    }
  }
  return expected;
}

TEST(ToCostmapUpdateMsgTest, CutsOutTheRectangleTheCallerAsked)
{
  const eltanin::map::Costmap costmap = make_numbered_costmap();
  const CellRect rect{1, 1, 2, 2};
  const auto msg = to_costmap_update_msg(costmap, rect, "map", make_stamp(7, 8));
  ASSERT_TRUE(msg.ok()) << msg.error();
  EXPECT_EQ(msg.value().x, 1u);
  EXPECT_EQ(msg.value().y, 1u);
  EXPECT_EQ(msg.value().width, 2u);
  EXPECT_EQ(msg.value().height, 2u);
  EXPECT_EQ(msg.value().data.size(), msg.value().width * msg.value().height);
  EXPECT_EQ(msg.value().data, cut_out(costmap, rect));
  EXPECT_EQ(msg.value().header.frame_id, "map");
  EXPECT_EQ(msg.value().header.stamp.sec, 7);
  EXPECT_EQ(msg.value().header.stamp.nanosec, 8u);
}

TEST(ToCostmapUpdateMsgTest, KeepsTheRowOrderSoAPatchIsNotTransposed)
{
  const eltanin::map::Costmap costmap = make_numbered_costmap();
  const auto msg = to_costmap_update_msg(costmap, CellRect{1, 0, 2, 1}, "map", make_stamp(0, 0));
  ASSERT_TRUE(msg.ok()) << msg.error();
  const std::vector<std::uint8_t> expected{2, 3, 6, 7};
  EXPECT_EQ(msg.value().data, expected);
}

TEST(ToCostmapUpdateMsgTest, AFullMapRectangleIsTheWholeCellVector)
{
  const eltanin::map::Costmap costmap = make_numbered_costmap();
  const auto msg = to_costmap_update_msg(costmap, CellRect{0, 0, 3, 2}, "map", make_stamp(0, 0));
  ASSERT_TRUE(msg.ok()) << msg.error();
  EXPECT_EQ(msg.value().data, costmap.data());
  EXPECT_EQ(msg.value().width, 4u);
  EXPECT_EQ(msg.value().height, 3u);
}

TEST(ToCostmapUpdateMsgTest, ASingleCellRectangleIsOneByte)
{
  const eltanin::map::Costmap costmap = make_numbered_costmap();
  const auto msg = to_costmap_update_msg(costmap, CellRect{3, 2, 3, 2}, "map", make_stamp(0, 0));
  ASSERT_TRUE(msg.ok()) << msg.error();
  const std::vector<std::uint8_t> expected{12};
  EXPECT_EQ(msg.value().data, expected);
}

TEST(ToCostmapUpdateMsgTest, RejectsARectangleThatDoesNotFitTheMap)
{
  const eltanin::map::Costmap costmap = make_numbered_costmap();
  const auto too_wide =
    to_costmap_update_msg(costmap, CellRect{0, 0, 4, 2}, "map", make_stamp(0, 0));
  EXPECT_FALSE(too_wide.ok());
  EXPECT_TRUE(contains(too_wide.error(), "does not fit a 4x3 map"));
  EXPECT_TRUE(contains(too_wide.error(), "max 4,2"));
  EXPECT_TRUE(is_one_line(too_wide.error()));

  const auto too_tall =
    to_costmap_update_msg(costmap, CellRect{0, 0, 3, 3}, "map", make_stamp(0, 0));
  EXPECT_FALSE(too_tall.ok());
  EXPECT_TRUE(contains(too_tall.error(), "does not fit a 4x3 map"));
}

TEST(ToCostmapUpdateMsgTest, RejectsARectangleBeforeTheFirstCell)
{
  const auto rejected =
    to_costmap_update_msg(make_numbered_costmap(), CellRect{-1, 0, 1, 1}, "map", make_stamp(0, 0));
  EXPECT_FALSE(rejected.ok());
  EXPECT_TRUE(contains(rejected.error(), "starts before the first cell"));
  EXPECT_TRUE(is_one_line(rejected.error()));
}

TEST(ToCostmapUpdateMsgTest, RejectsAnInvertedRectangleRatherThanWrappingAround)
{
  const auto rejected =
    to_costmap_update_msg(make_numbered_costmap(), CellRect{2, 0, 1, 1}, "map", make_stamp(0, 0));
  EXPECT_FALSE(rejected.ok());
  EXPECT_TRUE(contains(rejected.error(), "min must not exceed max"));
  EXPECT_TRUE(names_the_package_once(rejected.error()));
}

TEST(ToCostmapUpdateMsgTest, RejectsTheSameDegenerateMapsAsTheFullTopic)
{
  const auto no_geometry =
    to_costmap_update_msg(eltanin::map::Costmap{}, CellRect{0, 0, 0, 0}, "map", make_stamp(0, 0));
  EXPECT_FALSE(no_geometry.ok());
  EXPECT_TRUE(contains(no_geometry.error(), "resolution"));

  // MapGeometry now refuses zero dimensions, so no conversion can ever be handed such a map.
  EXPECT_THROW(make_costmap(0, 0, {}), std::invalid_argument);
}

TEST(ToCostmapUpdateMsgTest, CarriesTheReservedValuesTheVisualizationTopicCannot)
{
  const std::vector<std::uint8_t> raw{
    FREE_SPACE, INSCRIBED_INFLATED_OBSTACLE, LETHAL_OBSTACLE, NO_INFORMATION};
  const auto msg =
    to_costmap_update_msg(make_costmap(4, 1, raw), CellRect{0, 0, 3, 0}, "map", make_stamp(0, 0));
  ASSERT_TRUE(msg.ok()) << msg.error();
  EXPECT_EQ(msg.value().data, raw);
}

/// A patch built by hand, so a rejection can be tested without going through the producing side.
eltanin_msgs::msg::CostmapUpdate make_update_msg(
  std::uint32_t x, std::uint32_t y, std::uint32_t width, std::uint32_t height,
  const std::vector<std::uint8_t> & data)
{
  eltanin_msgs::msg::CostmapUpdate msg;
  msg.header.frame_id = "map";
  msg.x = x;
  msg.y = y;
  msg.width = width;
  msg.height = height;
  msg.data = data;
  return msg;
}

TEST(ApplyCostmapUpdateTest, WritesTheRectangleAndNothingElse)
{
  eltanin::map::Costmap costmap = make_numbered_costmap();
  const eltanin::map::Costmap before = costmap;
  const auto status = apply_costmap_update(make_update_msg(1, 1, 2, 2, {90, 91, 92, 93}), costmap);
  ASSERT_TRUE(status.ok()) << status.message();

  EXPECT_EQ(costmap.get(1, 1), 90);
  EXPECT_EQ(costmap.get(2, 1), 91);
  EXPECT_EQ(costmap.get(1, 2), 92);
  EXPECT_EQ(costmap.get(2, 2), 93);
  for (int my = 0; my < costmap.size_y(); ++my) {
    for (int mx = 0; mx < costmap.size_x(); ++mx) {
      const bool inside = mx >= 1 && mx <= 2 && my >= 1 && my <= 2;
      if (!inside) {
        EXPECT_EQ(costmap.get(mx, my), before.get(mx, my)) << "at (" << mx << ", " << my << ")";
      }
    }
  }
}

TEST(ApplyCostmapUpdateTest, KeepsTheRowOrderSoAPatchIsNotTransposed)
{
  eltanin::map::Costmap costmap = make_numbered_costmap();
  const auto status = apply_costmap_update(make_update_msg(1, 0, 2, 2, {90, 91, 92, 93}), costmap);
  ASSERT_TRUE(status.ok()) << status.message();
  EXPECT_EQ(costmap.get(1, 0), 90);
  EXPECT_EQ(costmap.get(2, 0), 91);
  EXPECT_EQ(costmap.get(1, 1), 92);
  EXPECT_EQ(costmap.get(2, 1), 93);
}

TEST(ApplyCostmapUpdateTest, CarriesTheReservedValues)
{
  eltanin::map::Costmap costmap = make_costmap(4, 1, {0, 0, 0, 0});
  const std::vector<std::uint8_t> raw{
    FREE_SPACE, INSCRIBED_INFLATED_OBSTACLE, LETHAL_OBSTACLE, NO_INFORMATION};
  const auto status = apply_costmap_update(make_update_msg(0, 0, 4, 1, raw), costmap);
  ASSERT_TRUE(status.ok()) << status.message();
  EXPECT_EQ(costmap.data(), raw);
}

TEST(ApplyCostmapUpdateTest, RejectsARectangleThatDoesNotFitTheMapWithoutWritingACell)
{
  eltanin::map::Costmap costmap = make_numbered_costmap();
  const eltanin::map::Costmap before = costmap;
  const auto status = apply_costmap_update(make_update_msg(3, 0, 2, 1, {90, 91}), costmap);
  EXPECT_FALSE(status.ok());
  EXPECT_TRUE(contains(status.message(), "does not fit a 4x3 map"));
  EXPECT_TRUE(is_one_line(status.message()));
  EXPECT_TRUE(names_the_package_once(status.message()));
  EXPECT_EQ(costmap.data(), before.data());
}

TEST(ApplyCostmapUpdateTest, RejectsADataLengthThatDisagreesWithTheRectangle)
{
  eltanin::map::Costmap costmap = make_numbered_costmap();
  const eltanin::map::Costmap before = costmap;
  const auto status = apply_costmap_update(make_update_msg(0, 0, 2, 2, {90, 91, 92}), costmap);
  EXPECT_FALSE(status.ok());
  EXPECT_TRUE(contains(status.message(), "data has 3 entries but the size implies 4"));
  EXPECT_TRUE(is_one_line(status.message()));
  EXPECT_EQ(costmap.data(), before.data());
}

TEST(ApplyCostmapUpdateTest, RejectsAnEmptyRectangle)
{
  eltanin::map::Costmap costmap = make_numbered_costmap();
  const auto no_width = apply_costmap_update(make_update_msg(0, 0, 0, 2, {}), costmap);
  EXPECT_FALSE(no_width.ok());
  EXPECT_TRUE(contains(no_width.message(), "width and height must both be non-zero"));

  const auto no_height = apply_costmap_update(make_update_msg(0, 0, 2, 0, {}), costmap);
  EXPECT_FALSE(no_height.ok());
  EXPECT_EQ(costmap.data(), make_numbered_costmap().data());
}

TEST(ApplyCostmapUpdateTest, RejectsALastCellThatOverflowsAnInt)
{
  eltanin::map::Costmap costmap = make_numbered_costmap();
  const auto status = apply_costmap_update(
    make_update_msg(std::numeric_limits<std::uint32_t>::max(), 0, 2, 1, {90, 91}), costmap);
  EXPECT_FALSE(status.ok());
  EXPECT_TRUE(contains(status.message(), "must fit in an int"));
  EXPECT_TRUE(is_one_line(status.message()));
}

TEST(ApplyCostmapUpdateTest, RejectsTheSameDegenerateMapsAsTheFullTopic)
{
  eltanin::map::Costmap no_geometry;
  const auto status = apply_costmap_update(make_update_msg(0, 0, 1, 1, {90}), no_geometry);
  EXPECT_FALSE(status.ok());
  EXPECT_TRUE(contains(status.message(), "resolution"));
}

TEST(ApplyCostmapUpdateTest, ARoundTripThroughAPatchRestoresTheCells)
{
  const eltanin::map::Costmap source = make_numbered_costmap();
  const CellRect rect{1, 0, 2, 2};
  const auto msg = to_costmap_update_msg(source, rect, "map", make_stamp(0, 0));
  ASSERT_TRUE(msg.ok()) << msg.error();

  eltanin::map::Costmap target = make_costmap(4, 3, std::vector<std::uint8_t>(12, 0));
  const auto status = apply_costmap_update(msg.value(), target);
  ASSERT_TRUE(status.ok()) << status.message();
  EXPECT_EQ(cut_out(target, rect), cut_out(source, rect));
}

}  // namespace
