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

#include "eltanin_ros_common/cost_conversion.hpp"
#include "test/conversion_test_helpers.hpp"

#include <eltanin/map/cost_values.hpp>

#include <gtest/gtest.h>

#include <cstdint>
#include <string>
#include <vector>

namespace
{

using eltanin::map::FREE_SPACE;
using eltanin::map::INSCRIBED_INFLATED_OBSTACLE;
using eltanin::map::LETHAL_OBSTACLE;
using eltanin::map::MAX_NON_OBSTACLE;
using eltanin::map::NO_INFORMATION;
using eltanin_ros_common::cost_to_occupancy;
using eltanin_ros_common::costs_to_occupancy;
using eltanin_ros_common::occupancy_to_cost;
using eltanin_ros_common::occupancy_to_costs;
using eltanin_ros_common::OccupancyThresholds;
using eltanin_ros_common::validate;
using eltanin_ros_common::test::contains;
using eltanin_ros_common::test::is_one_line;

std::uint8_t to_cost(int occupancy)
{
  return occupancy_to_cost(static_cast<std::int8_t>(occupancy), OccupancyThresholds{});
}

TEST(OccupancyThresholdsTest, DefaultsMatchTheDesignTable)
{
  const OccupancyThresholds thresholds;
  EXPECT_EQ(thresholds.occupied_threshold, 65);
  EXPECT_EQ(thresholds.free_threshold, 25);
  EXPECT_TRUE(validate(thresholds).ok());
}

TEST(OccupancyThresholdsTest, RejectsAnOutOfRangeOrInvertedPair)
{
  const auto negative = validate(OccupancyThresholds{65, -1});
  EXPECT_FALSE(negative.ok());
  EXPECT_TRUE(contains(negative.message(), "free_threshold"));

  const auto too_large = validate(OccupancyThresholds{101, 25});
  EXPECT_FALSE(too_large.ok());
  EXPECT_TRUE(contains(too_large.message(), "occupied_threshold"));

  const auto inverted = validate(OccupancyThresholds{25, 65});
  EXPECT_FALSE(inverted.ok());
  EXPECT_TRUE(contains(inverted.message(), "must be below"));

  EXPECT_FALSE(validate(OccupancyThresholds{25, 25}).ok());
}

TEST(OccupancyThresholdsTest, RejectionMessagesAreOneLine)
{
  const auto rejected = validate(OccupancyThresholds{25, 65});
  ASSERT_FALSE(rejected.ok());
  EXPECT_TRUE(is_one_line(rejected.message()));
}

TEST(OccupancyToCostTest, EveryNegativeValueIsUnknown)
{
  EXPECT_EQ(to_cost(-1), NO_INFORMATION);
  EXPECT_EQ(to_cost(-2), NO_INFORMATION);
  EXPECT_EQ(to_cost(-128), NO_INFORMATION);
}

TEST(OccupancyToCostTest, ThresholdBoundariesAreInclusive)
{
  EXPECT_EQ(to_cost(0), FREE_SPACE);
  EXPECT_EQ(to_cost(25), FREE_SPACE);
  EXPECT_EQ(to_cost(26), NO_INFORMATION);
  EXPECT_EQ(to_cost(64), NO_INFORMATION);
  EXPECT_EQ(to_cost(65), LETHAL_OBSTACLE);
  EXPECT_EQ(to_cost(100), LETHAL_OBSTACLE);
}

TEST(OccupancyToCostTest, ValuesAboveTheOccupancyRangeAreLethal)
{
  EXPECT_EQ(to_cost(127), LETHAL_OBSTACLE);
}

TEST(OccupancyToCostTest, TheMiddleBandIsUnknownRatherThanFree)
{
  for (int occupancy = 26; occupancy <= 64; ++occupancy) {
    EXPECT_EQ(to_cost(occupancy), NO_INFORMATION) << "occupancy " << occupancy;
  }
}

TEST(OccupancyToCostTest, OnlyTheThreeStaticMapValuesAreProduced)
{
  for (int occupancy = -128; occupancy <= 127; ++occupancy) {
    const std::uint8_t cost = to_cost(occupancy);
    const bool is_static_map_value =
      cost == FREE_SPACE || cost == LETHAL_OBSTACLE || cost == NO_INFORMATION;
    EXPECT_TRUE(is_static_map_value) << "occupancy " << occupancy << " gave cost " << int{cost};
  }
}

TEST(OccupancyToCostTest, CustomThresholdsMoveTheBoundaries)
{
  const OccupancyThresholds thresholds{50, 10};
  EXPECT_EQ(occupancy_to_cost(10, thresholds), FREE_SPACE);
  EXPECT_EQ(occupancy_to_cost(11, thresholds), NO_INFORMATION);
  EXPECT_EQ(occupancy_to_cost(49, thresholds), NO_INFORMATION);
  EXPECT_EQ(occupancy_to_cost(50, thresholds), LETHAL_OBSTACLE);
}

TEST(CostToOccupancyTest, ReservedValuesUseTheirOwnCodes)
{
  EXPECT_EQ(cost_to_occupancy(FREE_SPACE), 0);
  EXPECT_EQ(cost_to_occupancy(INSCRIBED_INFLATED_OBSTACLE), 99);
  EXPECT_EQ(cost_to_occupancy(LETHAL_OBSTACLE), 100);
  EXPECT_EQ(cost_to_occupancy(NO_INFORMATION), -1);
}

TEST(CostToOccupancyTest, InflationValuesScaleIntoOneToNinetyEight)
{
  EXPECT_EQ(cost_to_occupancy(1), 1);
  EXPECT_EQ(cost_to_occupancy(2), 1);
  EXPECT_EQ(cost_to_occupancy(3), 1);
  EXPECT_EQ(cost_to_occupancy(4), 2);
  EXPECT_EQ(cost_to_occupancy(MAX_NON_OBSTACLE), 98);
}

TEST(CostToOccupancyTest, IsMonotonicOverTheNonReservedRange)
{
  for (int cost = 0; cost < MAX_NON_OBSTACLE; ++cost) {
    const int lower = cost_to_occupancy(static_cast<std::uint8_t>(cost));
    const int higher = cost_to_occupancy(static_cast<std::uint8_t>(cost + 1));
    EXPECT_LE(lower, higher) << "cost " << cost << " to " << cost + 1;
  }
}

TEST(CostToOccupancyTest, ReservedCodesAreDisjointFromTheInflationRange)
{
  for (int cost = 1; cost <= MAX_NON_OBSTACLE; ++cost) {
    const int occupancy = cost_to_occupancy(static_cast<std::uint8_t>(cost));
    EXPECT_GE(occupancy, 1) << "cost " << cost;
    EXPECT_LE(occupancy, 98) << "cost " << cost;
  }
}

TEST(CostToOccupancyTest, InflationValuesCollapseSoTheFreeBoundaryIsNotRecoverable)
{
  EXPECT_EQ(cost_to_occupancy(1), cost_to_occupancy(3));
}

TEST(BulkConversionTest, MatchesTheScalarFormAndOverwritesTheOutput)
{
  const std::vector<std::int8_t> occupancy{-1, 0, 25, 26, 64, 65, 100};
  std::vector<std::uint8_t> costs{7, 7, 7};
  occupancy_to_costs(occupancy, OccupancyThresholds{}, costs);
  ASSERT_EQ(costs.size(), occupancy.size());
  for (std::size_t i = 0; i < occupancy.size(); ++i) {
    EXPECT_EQ(costs[i], occupancy_to_cost(occupancy[i], OccupancyThresholds{})) << "index " << i;
  }

  const std::vector<std::uint8_t> raw{
    FREE_SPACE, 1, MAX_NON_OBSTACLE, INSCRIBED_INFLATED_OBSTACLE, LETHAL_OBSTACLE, NO_INFORMATION};
  std::vector<std::int8_t> visualized{9, 9, 9, 9, 9, 9, 9, 9};
  costs_to_occupancy(raw, visualized);
  ASSERT_EQ(visualized.size(), raw.size());
  for (std::size_t i = 0; i < raw.size(); ++i) {
    EXPECT_EQ(visualized[i], cost_to_occupancy(raw[i])) << "index " << i;
  }
}

TEST(BulkConversionTest, EmptyInputGivesEmptyOutput)
{
  std::vector<std::uint8_t> costs{1, 2, 3};
  occupancy_to_costs({}, OccupancyThresholds{}, costs);
  EXPECT_TRUE(costs.empty());

  std::vector<std::int8_t> visualized{1, 2, 3};
  costs_to_occupancy({}, visualized);
  EXPECT_TRUE(visualized.empty());
}

}  // namespace
