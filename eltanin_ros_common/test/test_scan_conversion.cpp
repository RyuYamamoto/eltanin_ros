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

#include "eltanin_ros_common/scan_conversion.hpp"
#include "test/conversion_test_helpers.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <numbers>
#include <vector>

namespace
{

using eltanin_ros_common::to_scan;
using eltanin_ros_common::test::contains;
using eltanin_ros_common::test::is_one_line;

constexpr float INFINITE_RANGE = std::numeric_limits<float>::infinity();
constexpr float NOT_A_NUMBER = std::numeric_limits<float>::quiet_NaN();

sensor_msgs::msg::LaserScan make_scan(const std::vector<float> & ranges)
{
  sensor_msgs::msg::LaserScan msg;
  msg.header.frame_id = "laser_frame_from_the_message";
  msg.header.stamp.sec = 5;
  msg.header.stamp.nanosec = 6;
  msg.angle_min = -static_cast<float>(std::numbers::pi);
  msg.angle_max = static_cast<float>(std::numbers::pi);
  msg.angle_increment = 0.01F;
  msg.range_min = 0.1F;
  msg.range_max = 8.0F;
  msg.ranges = ranges;
  return msg;
}

TEST(ToScanTest, TakesTheFrameAndTimeFromTheMessageHeader)
{
  const auto result = to_scan(make_scan({1.0F, 2.0F}));
  ASSERT_TRUE(result.ok()) << result.error();
  EXPECT_EQ(result.value().frame_id, "laser_frame_from_the_message");
  EXPECT_EQ(result.value().stamp.sec, 5);
  EXPECT_EQ(result.value().stamp.nanosec, 6u);
}

TEST(ToScanTest, CopiesTheGeometryFields)
{
  const auto result = to_scan(make_scan({1.0F}));
  ASSERT_TRUE(result.ok()) << result.error();
  EXPECT_FLOAT_EQ(static_cast<float>(result.value().scan.angle_min), -std::numbers::pi_v<float>);
  EXPECT_FLOAT_EQ(static_cast<float>(result.value().scan.angle_increment), 0.01F);
  EXPECT_FLOAT_EQ(static_cast<float>(result.value().scan.range_min), 0.1F);
  EXPECT_FLOAT_EQ(static_cast<float>(result.value().scan.range_max), 8.0F);
}

TEST(ToScanTest, DoesNotUseAngleMaxBecauseScanDataCannotHoldIt)
{
  auto narrow = make_scan({1.0F, 2.0F, 3.0F});
  auto wide = narrow;
  wide.angle_max = 100.0F;
  const auto from_narrow = to_scan(narrow);
  const auto from_wide = to_scan(wide);
  ASSERT_TRUE(from_narrow.ok());
  ASSERT_TRUE(from_wide.ok());
  EXPECT_DOUBLE_EQ(from_narrow.value().scan.angle_min, from_wide.value().scan.angle_min);
  EXPECT_DOUBLE_EQ(
    from_narrow.value().scan.angle_increment, from_wide.value().scan.angle_increment);
  EXPECT_EQ(from_narrow.value().scan.ranges, from_wide.value().scan.ranges);
}

TEST(ToScanTest, CopiesInfiniteAndNaNRangesInsteadOfFilteringThem)
{
  const auto result = to_scan(make_scan({1.0F, INFINITE_RANGE, NOT_A_NUMBER, 0.05F, 20.0F}));
  ASSERT_TRUE(result.ok()) << result.error();
  const std::vector<float> & ranges = result.value().scan.ranges;
  ASSERT_EQ(ranges.size(), 5u);
  EXPECT_FLOAT_EQ(ranges[0], 1.0F);
  EXPECT_TRUE(std::isinf(ranges[1]));
  EXPECT_TRUE(std::isnan(ranges[2]));
  EXPECT_FLOAT_EQ(ranges[3], 0.05F);
  EXPECT_FLOAT_EQ(ranges[4], 20.0F);
}

TEST(ToScanTest, DoesNotModifyTheInputMessage)
{
  const auto original = make_scan({1.0F, INFINITE_RANGE});
  auto received = original;
  const auto result = to_scan(received);
  ASSERT_TRUE(result.ok()) << result.error();
  EXPECT_EQ(received.ranges.size(), original.ranges.size());
  EXPECT_FLOAT_EQ(received.ranges[0], original.ranges[0]);
  EXPECT_TRUE(std::isinf(received.ranges[1]));
  EXPECT_FLOAT_EQ(received.range_max, original.range_max);
}

TEST(ToScanTest, AcceptsAnEmptyScan)
{
  const auto result = to_scan(make_scan({}));
  ASSERT_TRUE(result.ok()) << result.error();
  EXPECT_TRUE(result.value().scan.ranges.empty());
  EXPECT_EQ(result.value().frame_id, "laser_frame_from_the_message");
}

TEST(ToScanTest, AcceptsAnUnboundedRangeMax)
{
  auto msg = make_scan({1.0F});
  msg.range_max = INFINITE_RANGE;
  const auto result = to_scan(msg);
  ASSERT_TRUE(result.ok()) << result.error();
  EXPECT_TRUE(std::isinf(result.value().scan.range_max));
}

TEST(ToScanTest, AcceptsAZeroAngleIncrement)
{
  auto msg = make_scan({1.0F});
  msg.angle_increment = 0.0F;
  const auto result = to_scan(msg);
  ASSERT_TRUE(result.ok()) << result.error();
  EXPECT_DOUBLE_EQ(result.value().scan.angle_increment, 0.0);
}

TEST(ToScanTest, RejectsNonFiniteAngles)
{
  auto angle_min = make_scan({1.0F});
  angle_min.angle_min = NOT_A_NUMBER;
  const auto rejected = to_scan(angle_min);
  EXPECT_FALSE(rejected.ok());
  EXPECT_TRUE(contains(rejected.error(), "angle_min"));
  EXPECT_TRUE(contains(rejected.error(), "laser_frame_from_the_message"));
  EXPECT_TRUE(is_one_line(rejected.error()));

  auto increment = make_scan({1.0F});
  increment.angle_increment = INFINITE_RANGE;
  EXPECT_FALSE(to_scan(increment).ok());
}

TEST(ToScanTest, RejectsABrokenRangeWindow)
{
  auto range_min = make_scan({1.0F});
  range_min.range_min = NOT_A_NUMBER;
  EXPECT_FALSE(to_scan(range_min).ok());

  auto range_max = make_scan({1.0F});
  range_max.range_max = NOT_A_NUMBER;
  const auto not_a_number = to_scan(range_max);
  EXPECT_FALSE(not_a_number.ok());
  EXPECT_TRUE(contains(not_a_number.error(), "range_max"));

  auto negative = make_scan({1.0F});
  negative.range_min = -0.1F;
  const auto below_zero = to_scan(negative);
  EXPECT_FALSE(below_zero.ok());
  EXPECT_TRUE(contains(below_zero.error(), "negative"));

  auto inverted = make_scan({1.0F});
  inverted.range_min = 9.0F;
  const auto above_max = to_scan(inverted);
  EXPECT_FALSE(above_max.ok());
  EXPECT_TRUE(contains(above_max.error(), "above range_max"));
}

}  // namespace
