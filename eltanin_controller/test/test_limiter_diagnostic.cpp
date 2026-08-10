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

#include "eltanin_controller/limiter_diagnostic.hpp"

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <gtest/gtest.h>

#include <set>
#include <string>

namespace
{

using eltanin_controller::forces_zero_command;
using eltanin_controller::level_of;
using eltanin_controller::LIMITER_REASONS;
using eltanin_controller::LimiterReason;
using eltanin_controller::name_of;
using Status = diagnostic_msgs::msg::DiagnosticStatus;

}  // namespace

TEST(LimiterReasonTest, EveryReasonHasItsOwnName)
{
  std::set<std::string> names;
  for (const LimiterReason reason : LIMITER_REASONS) {
    names.insert(name_of(reason));
  }
  EXPECT_EQ(names.size(), LIMITER_REASONS.size());
  EXPECT_STREQ(name_of(LimiterReason::None), "none");
  EXPECT_STREQ(name_of(LimiterReason::OutputDisabled), "output_disabled");
  EXPECT_STREQ(name_of(LimiterReason::OutsideMap), "outside_map");
}

TEST(LimiterReasonTest, TheDecisionOrderIsTheDeclarationOrder)
{
  EXPECT_EQ(LIMITER_REASONS.front(), LimiterReason::OutputDisabled);
  EXPECT_EQ(LIMITER_REASONS.back(), LimiterReason::None);
  for (std::size_t index = 0; index + 1 < LIMITER_REASONS.size(); ++index) {
    EXPECT_LT(
      static_cast<int>(LIMITER_REASONS[index]), static_cast<int>(LIMITER_REASONS[index + 1]));
  }
}

TEST(LimiterLevelTest, OnlyAMiswiredInputIsAnError)
{
  EXPECT_EQ(level_of(LimiterReason::CommandRejected), Status::ERROR);
  EXPECT_EQ(level_of(LimiterReason::MapRejected), Status::ERROR);

  for (const LimiterReason reason :
       {LimiterReason::CommandMissing, LimiterReason::CommandStale, LimiterReason::MapMissing,
        LimiterReason::MapStale}) {
    EXPECT_EQ(level_of(reason), Status::STALE) << name_of(reason);
  }
  for (const LimiterReason reason :
       {LimiterReason::OutputDisabled, LimiterReason::NoTransform, LimiterReason::OutsideMap,
        LimiterReason::Limited}) {
    EXPECT_EQ(level_of(reason), Status::WARN) << name_of(reason);
  }
  EXPECT_EQ(level_of(LimiterReason::None), Status::OK);
}

TEST(LimiterLevelTest, OnlyTheTwoLimiterOutcomesCarryACommand)
{
  for (const LimiterReason reason : LIMITER_REASONS) {
    const bool carries = reason == LimiterReason::Limited || reason == LimiterReason::None;
    EXPECT_EQ(forces_zero_command(reason), !carries) << name_of(reason);
  }
}
