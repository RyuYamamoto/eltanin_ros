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

#include "eltanin_controller/limiter_parameters.hpp"

#include <gtest/gtest.h>

#include <cassert>
#include <limits>
#include <set>
#include <string>
#include <vector>

namespace
{

using eltanin::DistanceTraversabilityModel;
using eltanin_controller::limiter::declared_keys;
using eltanin_controller::limiter::LimiterParameters;
using eltanin_controller::limiter::validate;
using eltanin_ros_common::ConversionStatus;

constexpr double INFINITE = std::numeric_limits<double>::infinity();

/// The kachaka collision box: inscribed 0.12 m, circumscribed about 0.207 m.
eltanin::Polygon2D kachaka_footprint()
{
  return eltanin::Polygon2D{
    Eigen::Vector2d{-0.150, -0.120}, Eigen::Vector2d{0.237, -0.120}, Eigen::Vector2d{0.237, 0.120},
    Eigen::Vector2d{-0.150, 0.120}};
}

DistanceTraversabilityModel kachaka_model()
{
  const auto model = DistanceTraversabilityModel::from_footprint(kachaka_footprint(), 0.55);
  assert(model.has_value());
  return *model;
}

bool mentions(const ConversionStatus & status, const std::string & text)
{
  return status.message().find(text) != std::string::npos;
}

}  // namespace

TEST(LimiterParametersTest, TheDefaultsAreWhatEltaninShipsAndTheyValidate)
{
  const LimiterParameters parameters;
  const eltanin::collision::VelocityGovernorParams defaults;

  EXPECT_DOUBLE_EQ(parameters.update_frequency, 20.0);
  EXPECT_DOUBLE_EQ(parameters.cmd_timeout, 0.3);
  EXPECT_DOUBLE_EQ(parameters.map_timeout, 0.5);
  EXPECT_FALSE(parameters.output_enabled_on_startup);
  EXPECT_DOUBLE_EQ(parameters.distance_map.max_distance, 1.0);
  EXPECT_DOUBLE_EQ(parameters.governor.release_time, defaults.release_time);
  EXPECT_EQ(parameters.governor.limiter.prediction_steps, defaults.limiter.prediction_steps);
  EXPECT_DOUBLE_EQ(parameters.governor.limiter.reaction_time, defaults.limiter.reaction_time);
  EXPECT_DOUBLE_EQ(parameters.governor.limiter.collision_margin, defaults.limiter.collision_margin);
  EXPECT_DOUBLE_EQ(parameters.governor.limiter.stop_clearance, defaults.limiter.stop_clearance);
  EXPECT_DOUBLE_EQ(
    parameters.governor.limiter.slow_down_clearance, defaults.limiter.slow_down_clearance);
  EXPECT_DOUBLE_EQ(
    parameters.governor.limiter.min_proximity_scale, defaults.limiter.min_proximity_scale);
  EXPECT_TRUE(parameters.governor.limiter.exact_footprint_check);

  EXPECT_TRUE(validate(parameters, kachaka_model()).ok());
}

TEST(LimiterParametersTest, EveryDeclaredKeyIsListedOnceAndNoneIsAProfileKey)
{
  const std::vector<std::string> keys = declared_keys();
  const std::set<std::string> distinct(keys.begin(), keys.end());

  EXPECT_EQ(distinct.size(), keys.size());
  EXPECT_EQ(keys.size(), 13u);
  for (const std::string & key : keys) {
    EXPECT_FALSE(key.starts_with("robot.")) << key;
    EXPECT_FALSE(key.starts_with("frames.")) << key;
  }
  EXPECT_EQ(distinct.count("update_frequency"), 1u);
  EXPECT_EQ(distinct.count("output_enabled_on_startup"), 1u);
  EXPECT_EQ(distinct.count("max_deceleration"), 0u);
}

TEST(LimiterParametersTest, ReportsTheFirstViolationInOrderWithTheKeyAndTheValue)
{
  LimiterParameters parameters;
  parameters.update_frequency = 0.0;
  parameters.cmd_timeout = -1.0;
  const ConversionStatus rate = validate(parameters, kachaka_model());
  ASSERT_FALSE(rate.ok());
  EXPECT_TRUE(mentions(rate, "update_frequency"));
  EXPECT_TRUE(mentions(rate, "0.000000"));
  EXPECT_FALSE(mentions(rate, "cmd_timeout"));

  parameters.update_frequency = 20.0;
  const ConversionStatus command = validate(parameters, kachaka_model());
  ASSERT_FALSE(command.ok());
  EXPECT_TRUE(mentions(command, "cmd_timeout"));

  parameters.cmd_timeout = 0.3;
  parameters.map_timeout = 0.0;
  const ConversionStatus map = validate(parameters, kachaka_model());
  ASSERT_FALSE(map.ok());
  EXPECT_TRUE(mentions(map, "map_timeout"));
}

TEST(LimiterParametersTest, RejectsEveryUnusableLimiterValue)
{
  const auto rejected = [](auto mutate) {
    LimiterParameters parameters;
    mutate(parameters);
    return validate(parameters, kachaka_model());
  };

  const ConversionStatus steps =
    rejected([](LimiterParameters & p) { p.governor.limiter.prediction_steps = 0; });
  EXPECT_FALSE(steps.ok());
  EXPECT_TRUE(mentions(steps, "prediction_steps"));

  const ConversionStatus reaction =
    rejected([](LimiterParameters & p) { p.governor.limiter.reaction_time = 0.0; });
  EXPECT_FALSE(reaction.ok());
  EXPECT_TRUE(mentions(reaction, "reaction_time"));

  const ConversionStatus margin =
    rejected([](LimiterParameters & p) { p.governor.limiter.collision_margin = -0.1; });
  EXPECT_FALSE(margin.ok());
  EXPECT_TRUE(mentions(margin, "collision_margin"));

  const ConversionStatus stop =
    rejected([](LimiterParameters & p) { p.governor.limiter.stop_clearance = -0.1; });
  EXPECT_FALSE(stop.ok());
  EXPECT_TRUE(mentions(stop, "stop_clearance"));

  const ConversionStatus slow = rejected([](LimiterParameters & p) {
    p.governor.limiter.slow_down_clearance = p.governor.limiter.stop_clearance;
  });
  EXPECT_FALSE(slow.ok());
  EXPECT_TRUE(mentions(slow, "slow_down_clearance"));
  EXPECT_TRUE(mentions(slow, "stop_clearance"));

  const ConversionStatus floor_at_zero =
    rejected([](LimiterParameters & p) { p.governor.limiter.min_proximity_scale = 0.0; });
  EXPECT_FALSE(floor_at_zero.ok());
  EXPECT_TRUE(mentions(floor_at_zero, "min_proximity_scale"));

  const ConversionStatus floor_above_one =
    rejected([](LimiterParameters & p) { p.governor.limiter.min_proximity_scale = 1.5; });
  EXPECT_FALSE(floor_above_one.ok());
  EXPECT_TRUE(mentions(floor_above_one, "min_proximity_scale"));

  const ConversionStatus release =
    rejected([](LimiterParameters & p) { p.governor.release_time = INFINITE; });
  EXPECT_FALSE(release.ok());
  EXPECT_TRUE(mentions(release, "release_time"));

  const ConversionStatus deceleration =
    rejected([](LimiterParameters & p) { p.governor.limiter.max_deceleration = 0.0; });
  EXPECT_FALSE(deceleration.ok());
  EXPECT_TRUE(mentions(deceleration, "robot.max_decel"));
}

TEST(LimiterParametersTest, TheSaturationHasToLeaveRoomForTheFreeBand)
{
  LimiterParameters parameters;
  parameters.distance_map.max_distance = kachaka_model().circumscribed_radius();

  const ConversionStatus status = validate(parameters, kachaka_model());

  ASSERT_FALSE(status.ok());
  EXPECT_TRUE(mentions(status, "clearance_max_distance"));
  EXPECT_TRUE(mentions(status, "circumscribed"));
}
