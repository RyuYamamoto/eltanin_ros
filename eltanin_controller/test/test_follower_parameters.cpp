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

#include "eltanin_controller/follower_parameters.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <numbers>
#include <string>

namespace
{

using eltanin_controller::apply_velocity_limits;
using eltanin_controller::FollowerParameters;
using eltanin_controller::name_of;
using eltanin_controller::PathSource;
using eltanin_controller::to_path_source;
using eltanin_controller::validate;

constexpr double NOT_A_NUMBER = std::numeric_limits<double>::quiet_NaN();

::testing::AssertionResult names(const std::string & text, const std::string & key)
{
  if (text.find('\n') != std::string::npos) {
    return ::testing::AssertionFailure() << "'" << text << "' contains a newline";
  }
  if (text.find(key) == std::string::npos) {
    return ::testing::AssertionFailure() << "'" << text << "' does not name '" << key << "'";
  }
  return ::testing::AssertionSuccess();
}

/// What the node holds after apply_velocity_limits(), which is what create() would be handed.
FollowerParameters make_parameters()
{
  FollowerParameters parameters;
  parameters.follower.pure_pursuit.max_angular_vel = 1.57;
  parameters.approach.max_angular_vel = 1.57;
  return parameters;
}

TEST(PathSourceTest, TheTwoNamesRoundTripAndNothingElseIsAccepted)
{
  EXPECT_EQ(to_path_source("path"), PathSource::Path);
  EXPECT_EQ(to_path_source("trajectory"), PathSource::Trajectory);
  EXPECT_FALSE(to_path_source("bogus").has_value());
  EXPECT_FALSE(to_path_source("").has_value());
  EXPECT_STREQ(name_of(PathSource::Path), "path");
  EXPECT_STREQ(name_of(PathSource::Trajectory), "trajectory");
}

TEST(ValidateTest, TheDefaultsAreUsable)
{
  const auto status = validate(make_parameters());
  EXPECT_TRUE(status.ok()) << status.message();
}

TEST(ValidateTest, TheUpdateFrequencyMustBePositiveAndFinite)
{
  FollowerParameters parameters = make_parameters();
  parameters.update_frequency = 0.0;
  EXPECT_TRUE(names(validate(parameters).message(), "update_frequency"));

  parameters.update_frequency = NOT_A_NUMBER;
  EXPECT_TRUE(names(validate(parameters).message(), "update_frequency"));

  parameters.update_frequency = 1.0 / 7200.0;
  EXPECT_TRUE(names(validate(parameters).message(), "update_frequency"));
}

TEST(ValidateTest, ATrajectoryWithoutADeadlineIsRefusedButAPathWithoutOneIsNot)
{
  FollowerParameters parameters = make_parameters();
  parameters.trajectory_timeout = 0.0;
  EXPECT_TRUE(validate(parameters).ok()) << "path_source is path, so the trajectory deadline idles";

  parameters.path_source = PathSource::Trajectory;
  EXPECT_TRUE(names(validate(parameters).message(), "trajectory_timeout"));
}

TEST(ValidateTest, APathDeadlineOfZeroIsNoDeadlineAndANegativeOneIsAnError)
{
  FollowerParameters parameters = make_parameters();
  parameters.path_timeout = 0.0;
  EXPECT_TRUE(validate(parameters).ok());

  parameters.path_timeout = -1.0;
  EXPECT_TRUE(names(validate(parameters).message(), "path_timeout"));
}

TEST(ValidateTest, RefusesExactlyWhatPurePursuitCreateRefuses)
{
  FollowerParameters parameters = make_parameters();
  parameters.follower.pure_pursuit.yaw_tolerance = std::numbers::pi;
  EXPECT_FALSE(validate(parameters).ok());
  EXPECT_FALSE(eltanin::control::PurePursuit::create(parameters.follower.pure_pursuit).has_value());

  parameters.follower.pure_pursuit.yaw_tolerance = std::numbers::pi - 1e-12;
  EXPECT_TRUE(validate(parameters).ok());
  EXPECT_TRUE(eltanin::control::PurePursuit::create(parameters.follower.pure_pursuit).has_value());

  parameters = make_parameters();
  parameters.follower.pure_pursuit.lookahead_time = 0.0;
  EXPECT_TRUE(validate(parameters).ok());
  EXPECT_TRUE(eltanin::control::PurePursuit::create(parameters.follower.pure_pursuit).has_value());

  parameters.follower.pure_pursuit.min_lookahead_dist = 0.0;
  EXPECT_TRUE(names(validate(parameters).message(), "min_lookahead_dist"));
  EXPECT_FALSE(eltanin::control::PurePursuit::create(parameters.follower.pure_pursuit).has_value());
}

TEST(ValidateTest, RefusesExactlyWhatGoalApproachCreateRefuses)
{
  FollowerParameters parameters = make_parameters();
  parameters.approach.approach_distance = parameters.approach.xy_goal_tolerance - 1e-6;
  EXPECT_TRUE(names(validate(parameters).message(), "approach_distance"));
  EXPECT_TRUE(names(validate(parameters).message(), "xy_goal_tolerance"));
  EXPECT_FALSE(eltanin::control::GoalApproach::create(parameters.approach).has_value());

  parameters.approach.approach_distance = parameters.approach.xy_goal_tolerance;
  EXPECT_TRUE(validate(parameters).ok());
  EXPECT_TRUE(eltanin::control::GoalApproach::create(parameters.approach).has_value());

  parameters = make_parameters();
  parameters.approach.yaw_goal_tolerance = std::numbers::pi;
  EXPECT_TRUE(names(validate(parameters).message(), "yaw_goal_tolerance"));
  EXPECT_FALSE(eltanin::control::GoalApproach::create(parameters.approach).has_value());

  parameters = make_parameters();
  parameters.approach.approach_decel = -1.0;
  EXPECT_TRUE(names(validate(parameters).message(), "approach_decel"));
  EXPECT_FALSE(eltanin::control::GoalApproach::create(parameters.approach).has_value());
}

TEST(ValidateTest, TheAngularLimitIsNamedAfterTheProfileKeyItComesFrom)
{
  FollowerParameters parameters = make_parameters();
  parameters.follower.pure_pursuit.max_angular_vel = 0.0;
  EXPECT_TRUE(names(validate(parameters).message(), "robot.max_angular_vel"));
}

TEST(VelocityLimitsTest, TheAngularLimitReachesBothGenerators)
{
  FollowerParameters parameters;
  eltanin_ros_common::VelocityLimits limits;
  limits.max_linear_vel = 1.0;
  limits.max_angular_vel = 1.57;

  const auto clamp = apply_velocity_limits(parameters, limits);
  EXPECT_FALSE(clamp.clamped);
  EXPECT_DOUBLE_EQ(parameters.follower.pure_pursuit.max_angular_vel, 1.57);
  EXPECT_DOUBLE_EQ(parameters.approach.max_angular_vel, 1.57);
  EXPECT_DOUBLE_EQ(parameters.follower.pure_pursuit.desired_linear_vel, 0.5);
}

TEST(VelocityLimitsTest, ACruiseSpeedAboveTheBodyLimitIsClampedRatherThanRefused)
{
  FollowerParameters parameters;
  parameters.follower.pure_pursuit.desired_linear_vel = 0.5;
  eltanin_ros_common::VelocityLimits limits;
  limits.max_linear_vel = 0.30;
  limits.max_angular_vel = 1.57;

  const auto clamp = apply_velocity_limits(parameters, limits);
  EXPECT_TRUE(clamp.clamped);
  EXPECT_DOUBLE_EQ(clamp.requested, 0.5);
  EXPECT_DOUBLE_EQ(clamp.applied, 0.30);
  EXPECT_DOUBLE_EQ(parameters.follower.pure_pursuit.desired_linear_vel, 0.30);
  EXPECT_TRUE(validate(parameters).ok());
}

}  // namespace
