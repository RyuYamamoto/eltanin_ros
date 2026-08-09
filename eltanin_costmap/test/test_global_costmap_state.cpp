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

#include "eltanin_costmap/global_costmap_state.hpp"
#include "eltanin_costmap/update_region.hpp"

#include <eltanin/core/footprint.hpp>
#include <eltanin/map/cost_values.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace
{

using eltanin::DistanceTraversabilityModel;
using eltanin::map::CellRect;
using eltanin::map::Costmap;
using eltanin::map::FREE_SPACE;
using eltanin::map::InflationCostModel;
using eltanin::map::LETHAL_OBSTACLE;
using eltanin::map::MapGeometry;
using eltanin::map::NO_INFORMATION;
using eltanin_costmap::accept_static_map;
using eltanin_costmap::GlobalCostmapState;
using eltanin_costmap::inflation_cells;
using eltanin_ros_common::MapLimits;
using eltanin_ros_common::OccupancyThresholds;

constexpr double RESOLUTION = 0.05;
constexpr int SIZE = 20;
constexpr double INFLATION_RADIUS = 0.15;
constexpr double COST_SCALING_FACTOR = 10.0;

/// A power-of-two resolution and an exact multiple of it, so no rounding hides the reach of r.
constexpr double EXACT_RESOLUTION = 0.0625;
constexpr double EXACT_INFLATION_RADIUS = 0.25;

InflationCostModel make_inflation(double inflation_radius, double inscribed = 0.02)
{
  const auto distance_model =
    DistanceTraversabilityModel::from_radii(inscribed, 2.0 * inscribed, inflation_radius);
  const auto model = InflationCostModel::create(*distance_model, COST_SCALING_FACTOR);
  return *model;
}

Costmap make_map(int size_x, int size_y, double resolution = RESOLUTION)
{
  return Costmap(MapGeometry(size_x, size_y, resolution, Eigen::Vector2d::Zero()), FREE_SPACE);
}

Costmap make_static_map()
{
  return make_map(SIZE, SIZE);
}

/// A window on the global grid carrying one occupied cell, which is what local_map publishes.
Costmap make_window(int at_x, int at_y, int size_x, int size_y)
{
  Costmap window(
    MapGeometry(size_x, size_y, RESOLUTION, Eigen::Vector2d{at_x * RESOLUTION, at_y * RESOLUTION}),
    FREE_SPACE);
  return window;
}

nav_msgs::msg::OccupancyGrid make_grid(std::uint32_t width, std::uint32_t height)
{
  nav_msgs::msg::OccupancyGrid msg;
  msg.header.frame_id = "map";
  msg.info.resolution = RESOLUTION;
  msg.info.width = width;
  msg.info.height = height;
  msg.info.origin.position.x = -1.0;
  msg.info.origin.position.y = 2.0;
  msg.data.assign(static_cast<std::size_t>(width) * height, 0);
  return msg;
}

::testing::AssertionResult contains(const std::string & text, const std::string & fragment)
{
  if (text.find(fragment) != std::string::npos) {
    return ::testing::AssertionSuccess();
  }
  return ::testing::AssertionFailure() << "'" << text << "' does not contain '" << fragment << "'";
}

::testing::AssertionResult is_one_line(const std::string & text)
{
  if (text.find('\n') == std::string::npos) {
    return ::testing::AssertionSuccess();
  }
  return ::testing::AssertionFailure() << "'" << text << "' contains a newline";
}

::testing::AssertionResult describes_the_map(const std::string & text)
{
  if (!contains(text, "frame_id='")) {
    return ::testing::AssertionFailure() << "'" << text << "' does not name the frame";
  }
  if (!contains(text, "resolution=")) {
    return ::testing::AssertionFailure() << "'" << text << "' does not name the resolution";
  }
  if (!contains(text, "2x2")) {
    return ::testing::AssertionFailure() << "'" << text << "' does not name the size";
  }
  return is_one_line(text);
}

TEST(AcceptStaticMapTest, AcceptsAMapInTheConfiguredFrameAndKeepsItsGeometry)
{
  const auto result = accept_static_map(make_grid(3, 2), "map", OccupancyThresholds{});
  ASSERT_TRUE(result.ok()) << result.error();
  EXPECT_EQ(result.value().size_x(), 3);
  EXPECT_EQ(result.value().size_y(), 2);
  EXPECT_DOUBLE_EQ(result.value().geometry().origin().x(), -1.0);
  EXPECT_DOUBLE_EQ(result.value().geometry().origin().y(), 2.0);
}

TEST(AcceptStaticMapTest, ReducesTheMapToTheThreeValuesAStaticMapCarries)
{
  auto msg = make_grid(4, 1);
  msg.data = {0, 100, -1, 40};
  const auto result = accept_static_map(msg, "map", OccupancyThresholds{});
  ASSERT_TRUE(result.ok()) << result.error();
  EXPECT_EQ(result.value().get(0, 0), FREE_SPACE);
  EXPECT_EQ(result.value().get(1, 0), LETHAL_OBSTACLE);
  EXPECT_EQ(result.value().get(2, 0), NO_INFORMATION);
  EXPECT_EQ(result.value().get(3, 0), NO_INFORMATION);
}

TEST(AcceptStaticMapTest, RejectsAMapInAnotherFrameAndSaysWhichOneWasExpected)
{
  auto msg = make_grid(2, 2);
  msg.header.frame_id = "odom";
  const auto rejected = accept_static_map(msg, "map", OccupancyThresholds{});
  ASSERT_FALSE(rejected.ok());
  EXPECT_TRUE(contains(rejected.error(), "frame_id must be 'map'"));
  EXPECT_TRUE(contains(rejected.error(), "frames.map"));
  EXPECT_TRUE(describes_the_map(rejected.error()));
}

/// A half turn about x, about y, and a yaw of 0.5 rad, written out so no tf2 call is needed here.
struct RotatedOrigin
{
  std::string axis;
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double w{1.0};
};

TEST(AcceptStaticMapTest, RejectsAnOriginRotationOnAnyOfTheThreeAxes)
{
  const std::vector<RotatedOrigin> rotations{
    {"roll", 1.0, 0.0, 0.0, 0.0},
    {"pitch", 0.0, 1.0, 0.0, 0.0},
    {"yaw", 0.0, 0.0, std::sin(0.25), std::cos(0.25)}};
  for (const RotatedOrigin & rotation : rotations) {
    auto msg = make_grid(2, 2);
    msg.info.origin.orientation.x = rotation.x;
    msg.info.origin.orientation.y = rotation.y;
    msg.info.origin.orientation.z = rotation.z;
    msg.info.origin.orientation.w = rotation.w;

    const auto rejected = accept_static_map(msg, "map", OccupancyThresholds{});
    EXPECT_FALSE(rejected.ok()) << "a " << rotation.axis << " rotation was accepted";
    EXPECT_TRUE(describes_the_map(rejected.error()));
  }
}

TEST(AcceptStaticMapTest, RejectsAZeroQuaternionOrigin)
{
  auto msg = make_grid(2, 2);
  msg.info.origin.orientation.w = 0.0;
  const auto rejected = accept_static_map(msg, "map", OccupancyThresholds{});
  ASSERT_FALSE(rejected.ok());
  EXPECT_TRUE(describes_the_map(rejected.error()));
}

TEST(AcceptStaticMapTest, RejectsAnUnusableResolutionOrSize)
{
  auto zero = make_grid(2, 2);
  zero.info.resolution = 0.0;
  EXPECT_FALSE(accept_static_map(zero, "map", OccupancyThresholds{}).ok());

  auto negative = make_grid(2, 2);
  negative.info.resolution = -RESOLUTION;
  EXPECT_FALSE(accept_static_map(negative, "map", OccupancyThresholds{}).ok());

  auto not_a_number = make_grid(2, 2);
  not_a_number.info.resolution = std::numeric_limits<float>::quiet_NaN();
  EXPECT_FALSE(accept_static_map(not_a_number, "map", OccupancyThresholds{}).ok());

  EXPECT_FALSE(accept_static_map(make_grid(0, 2), "map", OccupancyThresholds{}).ok());
}

TEST(AcceptStaticMapTest, RejectsADataLengthThatDisagreesWithTheSize)
{
  auto msg = make_grid(2, 2);
  msg.data.pop_back();
  const auto rejected = accept_static_map(msg, "map", OccupancyThresholds{});
  ASSERT_FALSE(rejected.ok());
  EXPECT_TRUE(is_one_line(rejected.error()));
}

TEST(AcceptStaticMapTest, RejectsAMapWithMoreCellsThanTheLimitAllows)
{
  EXPECT_FALSE(accept_static_map(make_grid(2, 2), "map", OccupancyThresholds{}, MapLimits{3}).ok());
}

TEST(GlobalCostmapStateTest, TakesTheGeometryOfTheStaticMapAndInflatesItsObstacles)
{
  Costmap static_map = make_static_map();
  ASSERT_TRUE(static_map.set(10, 10, LETHAL_OBSTACLE));
  GlobalCostmapState state(std::move(static_map), make_inflation(INFLATION_RADIUS), false);

  const auto outcome = state.update();
  EXPECT_EQ(state.geometry().size_x(), SIZE);
  EXPECT_EQ(state.geometry().size_y(), SIZE);
  EXPECT_DOUBLE_EQ(state.geometry().resolution(), RESOLUTION);
  EXPECT_EQ(state.costmap().get(10, 10), LETHAL_OBSTACLE);
  EXPECT_GT(state.costmap().get(11, 10).value(), FREE_SPACE);
  EXPECT_GT(state.costmap().get(10, 11).value(), FREE_SPACE);
  EXPECT_EQ(state.costmap().get(0, 0), FREE_SPACE);
  EXPECT_EQ(outcome.observation_count, 0u);
}

TEST(GlobalCostmapStateTest, HasNoPatchToPublishWhenNothingWasObserved)
{
  GlobalCostmapState state(make_static_map(), make_inflation(INFLATION_RADIUS), false);
  EXPECT_FALSE(state.update().publish_rect.has_value());
}

TEST(GlobalCostmapStateTest, ASecondUpdateWithNoNewObservationsHasNoPatchAndTheSameMaster)
{
  Costmap static_map = make_static_map();
  ASSERT_TRUE(static_map.set(4, 4, LETHAL_OBSTACLE));
  GlobalCostmapState state(std::move(static_map), make_inflation(INFLATION_RADIUS), false);

  state.update();
  const std::vector<std::uint8_t> first = state.costmap().data();
  const auto second = state.update();
  EXPECT_FALSE(second.publish_rect.has_value());
  EXPECT_EQ(state.costmap().data(), first);
}

TEST(GlobalCostmapStateTest, AnAbsorbedWindowBecomesLethalAndYieldsAPatch)
{
  GlobalCostmapState state(make_static_map(), make_inflation(INFLATION_RADIUS), false);
  Costmap window = make_window(8, 9, 2, 2);
  ASSERT_TRUE(window.set(0, 0, LETHAL_OBSTACLE));
  ASSERT_EQ(state.absorb(window).discovered, 1u);

  const auto outcome = state.update();
  EXPECT_EQ(outcome.observation_count, 1u);
  EXPECT_EQ(state.costmap().get(8, 9), LETHAL_OBSTACLE);
  EXPECT_GT(state.costmap().get(9, 9).value(), FREE_SPACE);
  ASSERT_TRUE(outcome.publish_rect.has_value());
  EXPECT_LE(outcome.publish_rect->min_x, 8 - state.inflation_cells());
  EXPECT_GE(outcome.publish_rect->max_x, 8 + state.inflation_cells());
}

TEST(GlobalCostmapStateTest, ThePatchCoversEveryCellThatActuallyChanged)
{
  GlobalCostmapState state(make_static_map(), make_inflation(INFLATION_RADIUS), false);
  state.update();
  const std::vector<std::uint8_t> before = state.costmap().data();

  Costmap window = make_window(6, 6, 3, 3);
  ASSERT_TRUE(window.set(1, 1, LETHAL_OBSTACLE));
  ASSERT_EQ(state.absorb(window).discovered, 1u);

  const auto outcome = state.update();
  ASSERT_TRUE(outcome.publish_rect.has_value());
  const CellRect & patch = *outcome.publish_rect;
  const std::vector<std::uint8_t> & after = state.costmap().data();
  ASSERT_EQ(after.size(), before.size());

  std::size_t changed = 0;
  for (int my = 0; my < SIZE; ++my) {
    for (int mx = 0; mx < SIZE; ++mx) {
      const std::size_t at = state.geometry().index(mx, my);
      if (after[at] == before[at]) {
        continue;
      }
      ++changed;
      EXPECT_TRUE(mx >= patch.min_x && mx <= patch.max_x && my >= patch.min_y && my <= patch.max_y)
        << "cell (" << mx << ", " << my << ") changed but is outside the published patch";
    }
  }
  EXPECT_GT(changed, 0u);
}

TEST(GlobalCostmapStateTest, ClearingTheObservationsGoesBackToTheStaticMapAndItsInflation)
{
  Costmap static_map = make_static_map();
  ASSERT_TRUE(static_map.set(2, 2, LETHAL_OBSTACLE));
  GlobalCostmapState state(std::move(static_map), make_inflation(INFLATION_RADIUS), false);
  state.update();
  const std::vector<std::uint8_t> static_only = state.costmap().data();

  Costmap window = make_window(12, 13, 2, 2);
  ASSERT_TRUE(window.set(0, 0, LETHAL_OBSTACLE));
  ASSERT_EQ(state.absorb(window).discovered, 1u);
  state.update();
  ASSERT_NE(state.costmap().data(), static_only);

  state.clear_observations();
  EXPECT_EQ(state.observation_count(), 0u);
  const auto outcome = state.update();
  EXPECT_EQ(state.costmap().data(), static_only);
  ASSERT_TRUE(outcome.publish_rect.has_value());
  EXPECT_LE(outcome.publish_rect->min_x, 12);
  EXPECT_GE(outcome.publish_rect->max_x, 12);
  EXPECT_LE(outcome.publish_rect->min_y, 13);
  EXPECT_GE(outcome.publish_rect->max_y, 13);
}

TEST(GlobalCostmapStateTest, InflateUnknownDecidesWhetherUnknownCellsTakeAnInflatedCost)
{
  for (const bool inflate_unknown : {false, true}) {
    Costmap static_map = make_static_map();
    ASSERT_TRUE(static_map.set(5, 5, LETHAL_OBSTACLE));
    ASSERT_TRUE(static_map.set(6, 5, NO_INFORMATION));
    GlobalCostmapState state(
      std::move(static_map), make_inflation(INFLATION_RADIUS), inflate_unknown);
    state.update();

    if (inflate_unknown) {
      EXPECT_LT(state.costmap().get(6, 5).value(), NO_INFORMATION);
      EXPECT_GT(state.costmap().get(6, 5).value(), FREE_SPACE);
    } else {
      EXPECT_EQ(state.costmap().get(6, 5), NO_INFORMATION);
    }
  }
}

TEST(GlobalCostmapStateTest, TheInflationReachIsExactlyTheCellCountUpdateRegionReports)
{
  const int radius = inflation_cells(EXACT_INFLATION_RADIUS, EXACT_RESOLUTION);
  ASSERT_EQ(radius, 4);

  const int centre = 10;
  Costmap static_map = make_map(21, 21, EXACT_RESOLUTION);
  ASSERT_TRUE(static_map.set(centre, centre, LETHAL_OBSTACLE));
  GlobalCostmapState state(
    std::move(static_map), make_inflation(EXACT_INFLATION_RADIUS, 0.0625), false);
  ASSERT_EQ(state.inflation_cells(), radius);
  state.update();

  EXPECT_GT(state.costmap().get(centre + radius, centre).value(), FREE_SPACE);
  EXPECT_GT(state.costmap().get(centre - radius, centre).value(), FREE_SPACE);
  EXPECT_GT(state.costmap().get(centre, centre + radius).value(), FREE_SPACE);
  EXPECT_GT(state.costmap().get(centre, centre - radius).value(), FREE_SPACE);
  EXPECT_EQ(state.costmap().get(centre + radius + 1, centre), FREE_SPACE);
  EXPECT_EQ(state.costmap().get(centre - radius - 1, centre), FREE_SPACE);
  EXPECT_EQ(state.costmap().get(centre, centre + radius + 1), FREE_SPACE);
  EXPECT_EQ(state.costmap().get(centre, centre - radius - 1), FREE_SPACE);
}

}  // namespace
