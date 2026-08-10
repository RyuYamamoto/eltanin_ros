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

#include "eltanin_ros_common/stale_input.hpp"

#include <eltanin/map/grid_map.hpp>
#include <eltanin/map/map_geometry.hpp>
#include <rclcpp/time.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>

namespace eltanin_ros_common::test
{
namespace
{

constexpr std::int64_t SECOND_NS = 1000000000;
constexpr std::int64_t BASE_NS = 1700000000 * SECOND_NS;
constexpr double TIMEOUT = 0.5;

/// Counts value copies so the acceptance condition "a costmap is not copied per cycle" is testable.
struct CopyCounter
{
  static int copies;

  CopyCounter() = default;
  CopyCounter(const CopyCounter &) { ++copies; }
  CopyCounter(CopyCounter &&) noexcept = default;
  CopyCounter & operator=(const CopyCounter &)
  {
    ++copies;
    return *this;
  }
  CopyCounter & operator=(CopyCounter &&) noexcept = default;
  ~CopyCounter() = default;
};

int CopyCounter::copies = 0;

rclcpp::Time ros_time(std::int64_t nanoseconds)
{
  return rclcpp::Time(nanoseconds, RCL_ROS_TIME);
}

TEST(StaleInputTest, NeverUpdatedIsStale)
{
  const StaleInput<std::string> input(TIMEOUT);

  EXPECT_EQ(input.get(ros_time(BASE_NS)), nullptr);
  EXPECT_FALSE(input.elapsed_seconds(ros_time(BASE_NS)).has_value());
}

TEST(StaleInputTest, NeverUpdatedDoesNotCompareTimes)
{
  const StaleInput<std::string> input(TIMEOUT);

  EXPECT_NO_THROW({
    EXPECT_EQ(input.get(rclcpp::Time(BASE_NS, RCL_SYSTEM_TIME)), nullptr);
    EXPECT_EQ(input.get(rclcpp::Time(BASE_NS, RCL_STEADY_TIME)), nullptr);
  });
}

TEST(StaleInputTest, MixedClockTypesDoNotThrow)
{
  StaleInput<std::string> input(TIMEOUT);
  input.update("scan", rclcpp::Time(BASE_NS, RCL_SYSTEM_TIME));

  EXPECT_NO_THROW({
    const std::string * fresh = input.get(rclcpp::Time(BASE_NS + SECOND_NS / 10, RCL_ROS_TIME));
    ASSERT_NE(fresh, nullptr);
    EXPECT_EQ(*fresh, "scan");
    EXPECT_EQ(input.get(rclcpp::Time(BASE_NS + 10 * SECOND_NS, RCL_ROS_TIME)), nullptr);
  });
}

TEST(StaleInputTest, FreshValueIsReachable)
{
  StaleInput<std::string> input(TIMEOUT);
  input.update("scan", ros_time(BASE_NS));

  const std::string * value = input.get(ros_time(BASE_NS + SECOND_NS / 10));

  ASSERT_NE(value, nullptr);
  EXPECT_EQ(*value, "scan");
}

TEST(StaleInputTest, ElapsedExactlyTimeoutIsFresh)
{
  StaleInput<std::string> input(TIMEOUT);
  input.update("scan", ros_time(BASE_NS));

  EXPECT_NE(input.get(ros_time(BASE_NS + SECOND_NS / 2)), nullptr);
}

TEST(StaleInputTest, JustPastTimeoutIsStale)
{
  StaleInput<std::string> input(TIMEOUT);
  input.update("scan", ros_time(BASE_NS));

  EXPECT_EQ(input.get(ros_time(BASE_NS + SECOND_NS / 2 + 1)), nullptr);
}

TEST(StaleInputTest, FutureStampBeyondTimeoutIsStale)
{
  StaleInput<std::string> input(TIMEOUT);
  input.update("scan", ros_time(BASE_NS + 2 * SECOND_NS));

  EXPECT_EQ(input.get(ros_time(BASE_NS)), nullptr);
  const std::optional<double> elapsed = input.elapsed_seconds(ros_time(BASE_NS));
  ASSERT_TRUE(elapsed.has_value());
  EXPECT_NEAR(*elapsed, -2.0, 1e-6);
}

TEST(StaleInputTest, FutureStampWithinTimeoutIsFresh)
{
  StaleInput<std::string> input(TIMEOUT);
  input.update("scan", ros_time(BASE_NS + SECOND_NS / 10));

  EXPECT_NE(input.get(ros_time(BASE_NS)), nullptr);
}

TEST(StaleInputTest, BackwardsTimeRecoversOnNextUpdate)
{
  StaleInput<std::string> input(TIMEOUT);
  input.update("scan", ros_time(BASE_NS + 2 * SECOND_NS));
  ASSERT_EQ(input.get(ros_time(BASE_NS)), nullptr);

  input.update("scan", ros_time(BASE_NS));

  EXPECT_NE(input.get(ros_time(BASE_NS)), nullptr);
}

TEST(StaleInputTest, NonPositiveTimeoutIsAlwaysStale)
{
  for (const double timeout : {0.0, -1.0}) {
    StaleInput<std::string> input(timeout);
    input.update("scan", ros_time(BASE_NS));

    EXPECT_FALSE(input.timeout_is_usable());
    EXPECT_EQ(input.get(ros_time(BASE_NS)), nullptr);
  }
}

TEST(StaleInputTest, NonFiniteTimeoutIsAlwaysStale)
{
  for (const double timeout :
       {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity()}) {
    StaleInput<std::string> input(timeout);
    input.update("scan", ros_time(BASE_NS));

    EXPECT_FALSE(input.timeout_is_usable());
    EXPECT_EQ(input.get(ros_time(BASE_NS)), nullptr);
  }
}

TEST(StaleInputTest, UsableTimeoutIsReported)
{
  const StaleInput<std::string> input(TIMEOUT);

  EXPECT_TRUE(input.timeout_is_usable());
}

TEST(StaleInputTest, GetDoesNotCopyTheValue)
{
  CopyCounter::copies = 0;
  StaleInput<CopyCounter> input(TIMEOUT);
  input.update(CopyCounter{}, ros_time(BASE_NS));
  ASSERT_EQ(CopyCounter::copies, 0);

  for (int call = 0; call < 100; ++call) {
    EXPECT_NE(input.get(ros_time(BASE_NS)), nullptr);
  }

  EXPECT_EQ(CopyCounter::copies, 0);
}

TEST(StaleInputTest, UpdateReplacesValueAndStampTogether)
{
  StaleInput<std::string> input(TIMEOUT);
  input.update("first", ros_time(BASE_NS));
  ASSERT_EQ(input.get(ros_time(BASE_NS + 10 * SECOND_NS)), nullptr);

  input.update("second", ros_time(BASE_NS + 10 * SECOND_NS));

  const std::string * value = input.get(ros_time(BASE_NS + 10 * SECOND_NS));
  ASSERT_NE(value, nullptr);
  EXPECT_EQ(*value, "second");
}

TEST(StaleInputTest, ClearForgetsTheValueWithoutForgettingTheDeadline)
{
  StaleInput<std::string> input(TIMEOUT);
  input.update("first", ros_time(BASE_NS));
  ASSERT_NE(input.get(ros_time(BASE_NS)), nullptr);

  input.clear();

  EXPECT_EQ(input.get(ros_time(BASE_NS)), nullptr);
  EXPECT_FALSE(input.elapsed_seconds(ros_time(BASE_NS)).has_value());
  EXPECT_TRUE(input.timeout_is_usable());

  input.update("second", ros_time(BASE_NS));
  const std::string * value = input.get(ros_time(BASE_NS));
  ASSERT_NE(value, nullptr);
  EXPECT_EQ(*value, "second");
}

TEST(StaleInputTest, HoldsACostmap)
{
  const eltanin::map::MapGeometry geometry(4, 4, 0.05, Eigen::Vector2d::Zero());
  StaleInput<eltanin::map::Costmap> input(TIMEOUT);
  input.update(eltanin::map::Costmap(geometry, 7), ros_time(BASE_NS));

  const eltanin::map::Costmap * costmap = input.get(ros_time(BASE_NS));

  ASSERT_NE(costmap, nullptr);
  EXPECT_EQ(costmap->geometry(), geometry);
  EXPECT_EQ(costmap->get(0, 0), 7);
}

}  // namespace
}  // namespace eltanin_ros_common::test
