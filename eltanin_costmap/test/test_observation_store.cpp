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

#include "eltanin_costmap/observation_store.hpp"

#include <eltanin/map/cost_values.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <optional>
#include <utility>
#include <vector>

namespace
{

using eltanin::map::CellRect;
using eltanin::map::Costmap;
using eltanin::map::FREE_SPACE;
using eltanin::map::INSCRIBED_INFLATED_OBSTACLE;
using eltanin::map::LETHAL_OBSTACLE;
using eltanin::map::MapGeometry;
using eltanin::map::NO_INFORMATION;
using eltanin_costmap::ObservationStore;

constexpr double RESOLUTION = 0.05;
constexpr int SIZE = 10;

Costmap make_map(int size_x, int size_y, const Eigen::Vector2d & origin, double resolution)
{
  return Costmap(MapGeometry(size_x, size_y, resolution, origin), FREE_SPACE);
}

Costmap make_static_map()
{
  return make_map(SIZE, SIZE, Eigen::Vector2d::Zero(), RESOLUTION);
}

/// A window placed on the global grid by cell, which is what local_map is expected to publish.
Costmap make_window(int at_x, int at_y, int size_x, int size_y, double resolution = RESOLUTION)
{
  return make_map(
    size_x, size_y, Eigen::Vector2d{at_x * RESOLUTION, at_y * RESOLUTION}, resolution);
}

Eigen::Vector2d cell_centre(int mx, int my)
{
  return make_static_map().geometry().map_to_world(mx, my);
}

::testing::AssertionResult is_cell_centre(const Eigen::Vector2d & point, int mx, int my)
{
  const Eigen::Vector2d expected = cell_centre(mx, my);
  if ((point - expected).norm() < 1e-12) {
    return ::testing::AssertionSuccess();
  }
  return ::testing::AssertionFailure()
         << "point (" << point.x() << ", " << point.y() << ") is not the centre of cell (" << mx
         << ", " << my << ") at (" << expected.x() << ", " << expected.y() << ")";
}

::testing::AssertionResult equals(const CellRect & actual, const CellRect & expected)
{
  if (
    actual.min_x == expected.min_x && actual.min_y == expected.min_y &&
    actual.max_x == expected.max_x && actual.max_y == expected.max_y) {
    return ::testing::AssertionSuccess();
  }
  return ::testing::AssertionFailure()
         << "got (" << actual.min_x << "," << actual.min_y << " .. " << actual.max_x << ","
         << actual.max_y << "), expected (" << expected.min_x << "," << expected.min_y << " .. "
         << expected.max_x << "," << expected.max_y << ")";
}

::testing::AssertionResult contains_rect(const CellRect & outer, const CellRect & inner)
{
  if (
    outer.min_x <= inner.min_x && outer.min_y <= inner.min_y && outer.max_x >= inner.max_x &&
    outer.max_y >= inner.max_y) {
    return ::testing::AssertionSuccess();
  }
  return ::testing::AssertionFailure()
         << "(" << inner.min_x << "," << inner.min_y << " .. " << inner.max_x << "," << inner.max_y
         << ") is not inside (" << outer.min_x << "," << outer.min_y << " .. " << outer.max_x << ","
         << outer.max_y << ")";
}

/// The bbox of the accumulated points, computed from the points rather than from a window.
CellRect point_bounds(const ObservationStore & store, const MapGeometry & global)
{
  CellRect bounds{global.size_x(), global.size_y(), -1, -1};
  for (const Eigen::Vector2d & point : store.points()) {
    const auto at = global.world_to_map(point);
    bounds.min_x = std::min(bounds.min_x, at->x);
    bounds.min_y = std::min(bounds.min_y, at->y);
    bounds.max_x = std::max(bounds.max_x, at->x);
    bounds.max_y = std::max(bounds.max_y, at->y);
  }
  return bounds;
}

TEST(ObservationStoreTest, StoresACellTheStaticMapDoesNotHaveAsAGlobalCellCentre)
{
  ObservationStore store(make_static_map());
  Costmap window = make_window(3, 4, 2, 2);
  ASSERT_TRUE(window.set(0, 0, LETHAL_OBSTACLE));

  const auto outcome = store.absorb(window);
  EXPECT_TRUE(outcome.overlaps);
  EXPECT_FALSE(outcome.resolution_differs);
  EXPECT_EQ(outcome.discovered, 1u);
  ASSERT_EQ(store.size(), 1u);
  EXPECT_TRUE(is_cell_centre(store.points()[0], 3, 4));
}

TEST(ObservationStoreTest, IgnoresACellTheStaticMapAlreadyKnowsIsOccupied)
{
  Costmap static_map = make_static_map();
  ASSERT_TRUE(static_map.set(3, 4, LETHAL_OBSTACLE));
  ObservationStore store(std::move(static_map));

  Costmap window = make_window(3, 4, 2, 2);
  ASSERT_TRUE(window.set(0, 0, LETHAL_OBSTACLE));

  const auto outcome = store.absorb(window);
  EXPECT_TRUE(outcome.overlaps);
  EXPECT_EQ(outcome.discovered, 0u);
  EXPECT_EQ(store.size(), 0u);
}

TEST(ObservationStoreTest, StoresOnlyLethalCellsAndNotTheInflatedOrUnknownOnes)
{
  ObservationStore store(make_static_map());
  Costmap window = make_window(0, 0, 4, 1);
  ASSERT_TRUE(window.set(0, 0, FREE_SPACE));
  ASSERT_TRUE(window.set(1, 0, NO_INFORMATION));
  ASSERT_TRUE(window.set(2, 0, INSCRIBED_INFLATED_OBSTACLE));
  ASSERT_TRUE(window.set(3, 0, LETHAL_OBSTACLE));

  EXPECT_EQ(store.absorb(window).discovered, 1u);
  ASSERT_EQ(store.size(), 1u);
  EXPECT_TRUE(is_cell_centre(store.points()[0], 3, 0));
}

TEST(ObservationStoreTest, DeduplicatesACellSeenInTwoWindows)
{
  ObservationStore store(make_static_map());
  Costmap window = make_window(2, 2, 2, 2);
  ASSERT_TRUE(window.set(1, 1, LETHAL_OBSTACLE));

  EXPECT_EQ(store.absorb(window).discovered, 1u);
  EXPECT_EQ(store.absorb(window).discovered, 0u);
  EXPECT_EQ(store.size(), 1u);
}

TEST(ObservationStoreTest, KeepsDiscoveryOrderAcrossWindowsRatherThanCellOrder)
{
  ObservationStore store(make_static_map());
  Costmap later_cell = make_window(7, 8, 1, 1);
  ASSERT_TRUE(later_cell.set(0, 0, LETHAL_OBSTACLE));
  Costmap earlier_cell = make_window(1, 0, 1, 1);
  ASSERT_TRUE(earlier_cell.set(0, 0, LETHAL_OBSTACLE));

  ASSERT_EQ(store.absorb(later_cell).discovered, 1u);
  ASSERT_EQ(store.absorb(earlier_cell).discovered, 1u);
  ASSERT_EQ(store.size(), 2u);
  EXPECT_TRUE(is_cell_centre(store.points()[0], 7, 8));
  EXPECT_TRUE(is_cell_centre(store.points()[1], 1, 0));
}

TEST(ObservationStoreTest, WithinOneWindowTheOrderIsRowMajorAndThereforeRepeatable)
{
  ObservationStore store(make_static_map());
  Costmap window = make_window(0, 0, 3, 3);
  ASSERT_TRUE(window.set(2, 2, LETHAL_OBSTACLE));
  ASSERT_TRUE(window.set(0, 1, LETHAL_OBSTACLE));
  ASSERT_TRUE(window.set(1, 0, LETHAL_OBSTACLE));

  ASSERT_EQ(store.absorb(window).discovered, 3u);
  EXPECT_TRUE(is_cell_centre(store.points()[0], 1, 0));
  EXPECT_TRUE(is_cell_centre(store.points()[1], 0, 1));
  EXPECT_TRUE(is_cell_centre(store.points()[2], 2, 2));
}

TEST(ObservationStoreTest, TheChangeRegionIsTheUnionOfTheWindowsAndCoversTheStoredPoints)
{
  ObservationStore store(make_static_map());
  Costmap first = make_window(1, 1, 2, 2);
  ASSERT_TRUE(first.set(0, 0, LETHAL_OBSTACLE));
  Costmap second = make_window(6, 7, 3, 2);
  ASSERT_TRUE(second.set(2, 1, LETHAL_OBSTACLE));

  ASSERT_TRUE(store.absorb(first).overlaps);
  ASSERT_TRUE(store.absorb(second).overlaps);

  const auto region = store.take_change_region();
  ASSERT_TRUE(region.has_value());
  EXPECT_TRUE(equals(*region, CellRect{1, 1, 8, 8}));
  EXPECT_TRUE(contains_rect(*region, point_bounds(store, make_static_map().geometry())));
}

TEST(ObservationStoreTest, TakingTheChangeRegionTwiceLeavesNothingTheSecondTime)
{
  ObservationStore store(make_static_map());
  Costmap window = make_window(1, 1, 2, 2);
  ASSERT_TRUE(window.set(0, 0, LETHAL_OBSTACLE));
  ASSERT_TRUE(store.absorb(window).overlaps);

  EXPECT_TRUE(store.take_change_region().has_value());
  EXPECT_FALSE(store.take_change_region().has_value());
}

TEST(ObservationStoreTest, AWindowOutsideTheStaticMapChangesNothing)
{
  ObservationStore store(make_static_map());
  Costmap window = make_map(2, 2, Eigen::Vector2d{5.0, 5.0}, RESOLUTION);
  ASSERT_TRUE(window.set(0, 0, LETHAL_OBSTACLE));

  const auto outcome = store.absorb(window);
  EXPECT_FALSE(outcome.overlaps);
  EXPECT_EQ(outcome.discovered, 0u);
  EXPECT_EQ(store.size(), 0u);
  EXPECT_FALSE(store.take_change_region().has_value());
}

TEST(ObservationStoreTest, AHalfOverlappingWindowKeepsTheCellsInsideAndClampsTheRectangle)
{
  ObservationStore store(make_static_map());
  Costmap window = make_window(8, 0, 4, 1);
  ASSERT_TRUE(window.set(0, 0, LETHAL_OBSTACLE));
  ASSERT_TRUE(window.set(1, 0, LETHAL_OBSTACLE));
  ASSERT_TRUE(window.set(2, 0, LETHAL_OBSTACLE));
  ASSERT_TRUE(window.set(3, 0, LETHAL_OBSTACLE));

  const auto outcome = store.absorb(window);
  EXPECT_TRUE(outcome.overlaps);
  EXPECT_EQ(outcome.discovered, 2u);
  EXPECT_TRUE(is_cell_centre(store.points()[0], 8, 0));
  EXPECT_TRUE(is_cell_centre(store.points()[1], 9, 0));

  const auto region = store.take_change_region();
  ASSERT_TRUE(region.has_value());
  EXPECT_TRUE(equals(*region, CellRect{8, 0, 9, 0}));
}

TEST(ObservationStoreTest, ACoarserWindowIsReportedAndStillAccumulated)
{
  ObservationStore store(make_static_map());
  Costmap window = make_map(2, 2, Eigen::Vector2d::Zero(), 2.0 * RESOLUTION);
  ASSERT_TRUE(window.set(0, 0, LETHAL_OBSTACLE));

  const auto outcome = store.absorb(window);
  EXPECT_TRUE(outcome.overlaps);
  EXPECT_TRUE(outcome.resolution_differs);
  EXPECT_EQ(outcome.discovered, 1u);
  ASSERT_EQ(store.size(), 1u);
  EXPECT_TRUE(is_cell_centre(store.points()[0], 1, 1));
}

TEST(ObservationStoreTest, AResolutionThatOnlyDiffersByFloat32NarrowingIsNotADisagreement)
{
  ObservationStore store(make_static_map());
  const double narrowed = static_cast<double>(static_cast<float>(RESOLUTION));
  ASSERT_NE(narrowed, RESOLUTION);
  Costmap window = make_map(2, 2, Eigen::Vector2d::Zero(), narrowed);

  EXPECT_FALSE(store.absorb(window).resolution_differs);
}

TEST(ObservationStoreTest, ClearDropsTheObservationsAndSaysWhereTheyWere)
{
  ObservationStore store(make_static_map());
  Costmap window = make_window(2, 3, 4, 4);
  ASSERT_TRUE(window.set(0, 0, LETHAL_OBSTACLE));
  ASSERT_TRUE(window.set(3, 3, LETHAL_OBSTACLE));
  ASSERT_EQ(store.absorb(window).discovered, 2u);
  ASSERT_TRUE(store.take_change_region().has_value());

  store.clear();
  EXPECT_EQ(store.size(), 0u);
  EXPECT_TRUE(store.points().empty());
  const auto region = store.take_change_region();
  ASSERT_TRUE(region.has_value());
  EXPECT_TRUE(equals(*region, CellRect{2, 3, 5, 6}));
}

TEST(ObservationStoreTest, ClearOnAnEmptyStoreLeavesTheChangeRegionEmpty)
{
  ObservationStore store(make_static_map());
  store.clear();
  EXPECT_FALSE(store.take_change_region().has_value());
}

TEST(ObservationStoreTest, ADegenerateWindowIsRefusedRatherThanIndexedIntoNothing)
{
  ObservationStore store(make_static_map());
  EXPECT_FALSE(store.absorb(Costmap{}).overlaps);
  EXPECT_FALSE(store.absorb(make_map(0, 4, Eigen::Vector2d::Zero(), RESOLUTION)).overlaps);
  EXPECT_FALSE(store.absorb(make_map(4, 0, Eigen::Vector2d::Zero(), RESOLUTION)).overlaps);
  EXPECT_FALSE(store.absorb(make_map(2, 2, Eigen::Vector2d::Zero(), 0.0)).overlaps);
  EXPECT_EQ(store.size(), 0u);
}

TEST(ObservationStoreTest, AWindowWhoseDataLengthDisagreesWithItsSizeIsRefused)
{
  ObservationStore store(make_static_map());
  Costmap window = make_window(0, 0, 2, 2);
  ASSERT_TRUE(window.set(0, 0, LETHAL_OBSTACLE));
  window.data().pop_back();

  EXPECT_FALSE(store.absorb(window).overlaps);
  EXPECT_EQ(store.size(), 0u);
}

}  // namespace
