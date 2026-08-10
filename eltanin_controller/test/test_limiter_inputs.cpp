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

#include "eltanin_controller/limiter_inputs.hpp"

#include <gtest/gtest.h>

#include <memory>
#include <string>

namespace
{

using eltanin_controller::CommandReading;
using eltanin_controller::LimiterInputs;
using eltanin_controller::LimiterReason;
using eltanin_controller::MapReading;
using eltanin_controller::SharedDistanceMap;

constexpr double CMD_TIMEOUT = 0.3;
constexpr double MAP_TIMEOUT = 0.5;

rclcpp::Time at(double seconds)
{
  return rclcpp::Time(static_cast<std::int64_t>(seconds * 1e9), RCL_ROS_TIME);
}

eltanin::Twist2D twist(double v)
{
  return eltanin::Twist2D{Eigen::Vector2d{v, 0.0}, 0.0};
}

SharedDistanceMap make_map()
{
  return std::make_shared<const eltanin::map::DistanceMap>(
    eltanin::map::MapGeometry(4, 4, 0.05, Eigen::Vector2d::Zero()), 1.0F);
}

}  // namespace

TEST(LimiterInputsTest, AnInputThatNeverArrivedIsMissingAndCarriesNoAge)
{
  const LimiterInputs inputs(CMD_TIMEOUT, MAP_TIMEOUT);

  const CommandReading command = inputs.read_command(at(10.0));
  const MapReading map = inputs.read_map(at(10.0));

  EXPECT_EQ(command.reason, LimiterReason::CommandMissing);
  EXPECT_FALSE(command.has_age);
  EXPECT_EQ(map.reason, LimiterReason::MapMissing);
  EXPECT_FALSE(map.has_age);
  EXPECT_EQ(map.value, nullptr);
}

TEST(LimiterInputsTest, AFreshInputIsAvailableAndComesBackByValue)
{
  LimiterInputs inputs(CMD_TIMEOUT, MAP_TIMEOUT);
  inputs.accept_command(twist(0.25), at(10.0));
  inputs.accept_map(make_map(), at(10.0));

  const CommandReading command = inputs.read_command(at(10.1));
  const MapReading map = inputs.read_map(at(10.1));

  EXPECT_EQ(command.reason, LimiterReason::None);
  EXPECT_DOUBLE_EQ(command.value.linear.x(), 0.25);
  EXPECT_TRUE(command.has_age);
  EXPECT_NEAR(command.age_seconds, 0.1, 1e-9);
  EXPECT_EQ(map.reason, LimiterReason::None);
  ASSERT_NE(map.value, nullptr);
  EXPECT_EQ(map.value->size_x(), 4);
}

TEST(LimiterInputsTest, TheDeadlineIsCheckedOnBothSides)
{
  LimiterInputs inputs(CMD_TIMEOUT, MAP_TIMEOUT);
  inputs.accept_command(twist(0.25), at(10.0));

  EXPECT_EQ(inputs.read_command(at(10.0 + CMD_TIMEOUT)).reason, LimiterReason::None);

  const CommandReading past = inputs.read_command(at(10.0 + CMD_TIMEOUT + 0.01));
  EXPECT_EQ(past.reason, LimiterReason::CommandStale);
  EXPECT_GT(past.age_seconds, CMD_TIMEOUT);

  // A stamp in the future is just as unusable as one that is too old.
  const CommandReading future = inputs.read_command(at(10.0 - CMD_TIMEOUT - 0.01));
  EXPECT_EQ(future.reason, LimiterReason::CommandStale);
  EXPECT_LT(future.age_seconds, 0.0);
}

TEST(LimiterInputsTest, TheTwoDeadlinesAreIndependent)
{
  LimiterInputs inputs(CMD_TIMEOUT, MAP_TIMEOUT);
  inputs.accept_command(twist(0.25), at(10.0));
  inputs.accept_map(make_map(), at(10.0));

  EXPECT_EQ(inputs.read_command(at(10.4)).reason, LimiterReason::CommandStale);
  EXPECT_EQ(inputs.read_map(at(10.4)).reason, LimiterReason::None);
  EXPECT_EQ(inputs.read_map(at(10.6)).reason, LimiterReason::MapStale);
}

TEST(LimiterInputsTest, ARejectionDropsTheHeldValueAndKeepsTheDetail)
{
  LimiterInputs inputs(CMD_TIMEOUT, MAP_TIMEOUT);
  inputs.accept_command(twist(0.25), at(10.0));
  inputs.accept_map(make_map(), at(10.0));

  inputs.reject_command("the command is in frame 'odom'");
  inputs.reject_map("the map is in frame 'odom'");

  const CommandReading command = inputs.read_command(at(10.0));
  const MapReading map = inputs.read_map(at(10.0));

  EXPECT_EQ(command.reason, LimiterReason::CommandRejected);
  EXPECT_DOUBLE_EQ(command.value.linear.x(), 0.0);
  EXPECT_EQ(map.reason, LimiterReason::MapRejected);
  EXPECT_EQ(map.value, nullptr);
  EXPECT_EQ(inputs.command_rejection(), "the command is in frame 'odom'");
  EXPECT_EQ(inputs.map_rejection(), "the map is in frame 'odom'");
}

TEST(LimiterInputsTest, AnAcceptedMessageClearsTheRejection)
{
  LimiterInputs inputs(CMD_TIMEOUT, MAP_TIMEOUT);
  inputs.reject_command("wrong frame");
  ASSERT_EQ(inputs.read_command(at(10.0)).reason, LimiterReason::CommandRejected);

  inputs.accept_command(twist(-0.1), at(10.0));

  EXPECT_EQ(inputs.read_command(at(10.0)).reason, LimiterReason::None);
  EXPECT_DOUBLE_EQ(inputs.read_command(at(10.0)).value.linear.x(), -0.1);
  EXPECT_TRUE(inputs.command_rejection().empty());
}

TEST(LimiterInputsTest, AnUnusableTimeoutMakesEveryReadStale)
{
  LimiterInputs inputs(0.0, MAP_TIMEOUT);
  inputs.accept_command(twist(0.25), at(10.0));

  EXPECT_EQ(inputs.read_command(at(10.0)).reason, LimiterReason::CommandStale);
}
