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

#include "eltanin_ros_common/diagnostic_conversion.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <string>

namespace
{

using eltanin_ros_common::DiagnosticStatusBuilder;
using eltanin_ros_common::to_diagnostic_value;
using Status = diagnostic_msgs::msg::DiagnosticStatus;

std::string value_of(const Status & status, const std::string & key)
{
  for (const diagnostic_msgs::msg::KeyValue & pair : status.values) {
    if (pair.key == key) {
      return pair.value;
    }
  }
  return "<missing>";
}

}  // namespace

TEST(DiagnosticValue, SpellsTheSpecialDoublesTheWayATestCanMatch)
{
  constexpr double INFINITY_VALUE = std::numeric_limits<double>::infinity();
  EXPECT_EQ(to_diagnostic_value(INFINITY_VALUE), "inf");
  EXPECT_EQ(to_diagnostic_value(-INFINITY_VALUE), "-inf");
  EXPECT_EQ(to_diagnostic_value(std::numeric_limits<double>::quiet_NaN()), "nan");
}

TEST(DiagnosticValue, DropsTheTrailingZerosOfAnOrdinaryDouble)
{
  EXPECT_EQ(to_diagnostic_value(0.0), "0");
  EXPECT_EQ(to_diagnostic_value(0.5), "0.5");
  EXPECT_EQ(to_diagnostic_value(-0.25), "-0.25");
  EXPECT_EQ(to_diagnostic_value(20.0), "20");
}

TEST(DiagnosticStatusBuilder, CarriesTheNameLevelAndMessage)
{
  const Status status =
    DiagnosticStatusBuilder("/collision_predictor", Status::WARN, "output is disabled").take();

  EXPECT_EQ(status.name, "/collision_predictor");
  EXPECT_EQ(status.level, Status::WARN);
  EXPECT_EQ(status.message, "output is disabled");
  EXPECT_TRUE(status.hardware_id.empty());
  EXPECT_TRUE(status.values.empty());
}

TEST(DiagnosticStatusBuilder, WritesEveryTypeWithOneSpelling)
{
  DiagnosticStatusBuilder builder("node", Status::OK, "");
  builder.add("reason", std::string("none"))
    .add("command_stale", false)
    .add("map_stale", true)
    .add("predicted_poses", std::size_t{11})
    .add("run_index", 3)
    .add("collision_distance", std::numeric_limits<double>::infinity())
    .add("cycle_dt", 0.05)
    .add("status", "tracking");
  const Status status = builder.take();

  ASSERT_EQ(status.values.size(), 8u);
  EXPECT_EQ(value_of(status, "reason"), "none");
  EXPECT_EQ(value_of(status, "command_stale"), "false");
  EXPECT_EQ(value_of(status, "map_stale"), "true");
  EXPECT_EQ(value_of(status, "predicted_poses"), "11");
  EXPECT_EQ(value_of(status, "run_index"), "3");
  EXPECT_EQ(value_of(status, "collision_distance"), "inf");
  EXPECT_EQ(value_of(status, "cycle_dt"), "0.05");
  EXPECT_EQ(value_of(status, "status"), "tracking");
}

TEST(DiagnosticStatusBuilder, KeepsTheInsertionOrder)
{
  DiagnosticStatusBuilder builder("node", Status::OK, "");
  builder.add("first", 1).add("second", 2).add("third", 3);
  const Status status = builder.take();

  ASSERT_EQ(status.values.size(), 3u);
  EXPECT_EQ(status.values[0].key, "first");
  EXPECT_EQ(status.values[1].key, "second");
  EXPECT_EQ(status.values[2].key, "third");
}
