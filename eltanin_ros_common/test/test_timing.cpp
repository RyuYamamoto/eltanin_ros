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

#include "eltanin_ros_common/timing.hpp"

#include <rclcpp/clock.hpp>

#include <gtest/gtest.h>
#include <rcl/time.h>

#include <cstdint>
#include <memory>

namespace eltanin_ros_common::test
{
namespace
{

constexpr std::int64_t SECOND_NS = 1000000000;

/// A ROS clock driven by rcl's time override, so every case moves time without sleeping.
rclcpp::Clock::SharedPtr make_overridden_clock()
{
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  EXPECT_EQ(rcl_enable_ros_time_override(clock->get_clock_handle()), RCL_RET_OK);
  return clock;
}

void set_time(const rclcpp::Clock::SharedPtr & clock, std::int64_t nanoseconds)
{
  ASSERT_EQ(rcl_set_ros_time_override(clock->get_clock_handle(), nanoseconds), RCL_RET_OK);
}

TEST(PeriodicClockTest, FirstTickHasNoPrevious)
{
  const auto clock = make_overridden_clock();
  set_time(clock, 10 * SECOND_NS);
  PeriodicClock periodic(clock);

  const PeriodicClock::Tick tick = periodic.tick();

  EXPECT_FALSE(tick.has_previous);
  EXPECT_FALSE(tick.usable());
}

TEST(PeriodicClockTest, MeasuresTheInjectedClock)
{
  const auto clock = make_overridden_clock();
  set_time(clock, 10 * SECOND_NS);
  PeriodicClock periodic(clock);
  periodic.tick();

  set_time(clock, 10 * SECOND_NS + SECOND_NS / 10);
  const PeriodicClock::Tick tick = periodic.tick();

  EXPECT_TRUE(tick.has_previous);
  EXPECT_TRUE(tick.usable());
  EXPECT_NEAR(tick.seconds, 0.1, 1e-9);
}

TEST(PeriodicClockTest, DoesNotUseWallTime)
{
  const auto clock = make_overridden_clock();
  set_time(clock, 10 * SECOND_NS);
  PeriodicClock periodic(clock);
  periodic.tick();

  const PeriodicClock::Tick tick = periodic.tick();

  EXPECT_EQ(tick.seconds, 0.0);
}

TEST(PeriodicClockTest, SameTimeTwiceIsNotUsable)
{
  const auto clock = make_overridden_clock();
  set_time(clock, 10 * SECOND_NS);
  PeriodicClock periodic(clock);
  periodic.tick();

  const PeriodicClock::Tick tick = periodic.tick();

  EXPECT_TRUE(tick.has_previous);
  EXPECT_EQ(tick.seconds, 0.0);
  EXPECT_FALSE(tick.usable());
}

TEST(PeriodicClockTest, ReferenceAdvancesEvenWhenNotUsable)
{
  const auto clock = make_overridden_clock();
  set_time(clock, 10 * SECOND_NS);
  PeriodicClock periodic(clock);
  periodic.tick();
  periodic.tick();

  set_time(clock, 10 * SECOND_NS + SECOND_NS / 10);
  const PeriodicClock::Tick tick = periodic.tick();

  EXPECT_NEAR(tick.seconds, 0.1, 1e-9);
}

TEST(PeriodicClockTest, BackwardsClockReturnsNegativeDt)
{
  const auto clock = make_overridden_clock();
  set_time(clock, 10 * SECOND_NS);
  PeriodicClock periodic(clock);
  periodic.tick();

  set_time(clock, 10 * SECOND_NS - 4 * SECOND_NS / 10);
  const PeriodicClock::Tick tick = periodic.tick();

  EXPECT_TRUE(tick.has_previous);
  EXPECT_NEAR(tick.seconds, -0.4, 1e-9);
  EXPECT_FALSE(tick.usable());
}

TEST(PeriodicClockTest, BackwardsClockRecoversOnTheNextTick)
{
  const auto clock = make_overridden_clock();
  set_time(clock, 10 * SECOND_NS);
  PeriodicClock periodic(clock);
  periodic.tick();
  set_time(clock, 10 * SECOND_NS - 4 * SECOND_NS / 10);
  periodic.tick();

  set_time(clock, 10 * SECOND_NS - 3 * SECOND_NS / 10);
  const PeriodicClock::Tick tick = periodic.tick();

  EXPECT_NEAR(tick.seconds, 0.1, 1e-9);
  EXPECT_TRUE(tick.usable());
}

TEST(PeriodicClockTest, LongGapIsNotClamped)
{
  const auto clock = make_overridden_clock();
  set_time(clock, 10 * SECOND_NS);
  PeriodicClock periodic(clock);
  periodic.tick();

  set_time(clock, 20 * SECOND_NS);
  const PeriodicClock::Tick tick = periodic.tick();

  EXPECT_NEAR(tick.seconds, 10.0, 1e-9);
  EXPECT_TRUE(tick.usable());
}

TEST(PeriodicClockTest, ResetMakesTheNextTickFirst)
{
  const auto clock = make_overridden_clock();
  set_time(clock, 10 * SECOND_NS);
  PeriodicClock periodic(clock);
  periodic.tick();

  periodic.reset();
  set_time(clock, 11 * SECOND_NS);
  const PeriodicClock::Tick tick = periodic.tick();

  EXPECT_FALSE(tick.has_previous);
  EXPECT_FALSE(tick.usable());
}

TEST(PeriodicClockTest, NullClockNeverProducesAUsableTick)
{
  PeriodicClock periodic(nullptr);

  EXPECT_FALSE(periodic.tick().usable());
  EXPECT_FALSE(periodic.tick().usable());
}

}  // namespace
}  // namespace eltanin_ros_common::test
