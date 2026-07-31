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

#include "eltanin_costmap/update_region.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <optional>

namespace
{

using eltanin::map::CellRect;
using eltanin::map::MapGeometry;
using eltanin_costmap::ChangeRegion;
using eltanin_costmap::inflation_cells;
using eltanin_costmap::publish_rect;

constexpr double RESOLUTION = 0.05;

MapGeometry make_geometry(int size_x, int size_y)
{
  return MapGeometry(size_x, size_y, RESOLUTION, Eigen::Vector2d{-1.0, 2.0});
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

TEST(InflationCellsTest, MatchesTheCeilOfTheRadiusOverTheResolution)
{
  EXPECT_EQ(inflation_cells(0.55, RESOLUTION), 11);
  EXPECT_EQ(inflation_cells(0.30, RESOLUTION), 6);
  EXPECT_EQ(inflation_cells(0.15, RESOLUTION), 3);
}

TEST(InflationCellsTest, RoundsUpSoASubCellRadiusStillReachesOneCell)
{
  EXPECT_EQ(inflation_cells(0.04, RESOLUTION), 1);
  EXPECT_EQ(inflation_cells(1e-9, RESOLUTION), 1);
}

TEST(InflationCellsTest, IsZeroWhenThereIsNothingToInflate)
{
  EXPECT_EQ(inflation_cells(0.0, RESOLUTION), 0);
  EXPECT_EQ(inflation_cells(-0.55, RESOLUTION), 0);
}

TEST(InflationCellsTest, IsZeroWhenTheResolutionIsUnusable)
{
  EXPECT_EQ(inflation_cells(0.55, 0.0), 0);
  EXPECT_EQ(inflation_cells(0.55, -RESOLUTION), 0);
  EXPECT_EQ(inflation_cells(0.55, std::numeric_limits<double>::quiet_NaN()), 0);
  EXPECT_EQ(inflation_cells(std::numeric_limits<double>::infinity(), RESOLUTION), 0);
}

TEST(InflationCellsTest, SaturatesRatherThanOverflowingTheCast)
{
  EXPECT_EQ(inflation_cells(1e300, 1e-300), std::numeric_limits<int>::max());
}

TEST(ChangeRegionTest, IsEmptyUntilSomethingIsAdded)
{
  ChangeRegion region;
  EXPECT_TRUE(region.empty());
  EXPECT_FALSE(region.rect().has_value());

  region.add(CellRect{3, 4, 3, 4});
  EXPECT_FALSE(region.empty());
  ASSERT_TRUE(region.rect().has_value());
  EXPECT_TRUE(equals(*region.rect(), CellRect{3, 4, 3, 4}));
}

TEST(ChangeRegionTest, TakesTheBoundingBoxOfTwoDisjointRectanglesNotTheirUnion)
{
  ChangeRegion region;
  region.add(CellRect{1, 1, 2, 2});
  region.add(CellRect{7, 8, 9, 9});
  EXPECT_TRUE(equals(*region.rect(), CellRect{1, 1, 9, 9}));
}

TEST(ChangeRegionTest, NeverShrinksSoTheOrderOfAdditionsDoesNotMatter)
{
  ChangeRegion forward;
  forward.add(CellRect{5, 5, 6, 6});
  forward.add(CellRect{0, 9, 1, 9});
  forward.add(CellRect{5, 5, 5, 5});

  ChangeRegion backward;
  backward.add(CellRect{5, 5, 5, 5});
  backward.add(CellRect{0, 9, 1, 9});
  backward.add(CellRect{5, 5, 6, 6});

  EXPECT_TRUE(equals(*forward.rect(), *backward.rect()));
  EXPECT_TRUE(equals(*forward.rect(), CellRect{0, 5, 6, 9}));
}

TEST(ChangeRegionTest, ClearGoesBackToEmptyAndNotToTheWholeMap)
{
  ChangeRegion region;
  region.add(CellRect{1, 1, 2, 2});
  region.clear();
  EXPECT_TRUE(region.empty());
  EXPECT_FALSE(region.rect().has_value());
}

TEST(PublishRectTest, NothingChangedStaysNothingRatherThanBecomingTheWholeMap)
{
  EXPECT_FALSE(publish_rect(std::nullopt, 11, make_geometry(20, 20)).has_value());
  EXPECT_FALSE(publish_rect(std::nullopt, 0, make_geometry(20, 20)).has_value());
}

TEST(PublishRectTest, ZeroCellsPublishesExactlyTheChangeRegion)
{
  const auto rect = publish_rect(CellRect{5, 5, 7, 8}, 0, make_geometry(20, 20));
  ASSERT_TRUE(rect.has_value());
  EXPECT_TRUE(equals(*rect, CellRect{5, 5, 7, 8}));
}

TEST(PublishRectTest, GrowsBySeveralCellsOnEverySide)
{
  const auto rect = publish_rect(CellRect{5, 5, 5, 5}, 2, make_geometry(20, 20));
  ASSERT_TRUE(rect.has_value());
  EXPECT_TRUE(equals(*rect, CellRect{3, 3, 7, 7}));
}

TEST(PublishRectTest, GrowsByExactlyTheInflationRadiusAndNotOneCellMoreOrLess)
{
  const int cells = inflation_cells(0.55, RESOLUTION);
  const auto rect = publish_rect(CellRect{40, 40, 40, 40}, cells, make_geometry(100, 100));
  ASSERT_TRUE(rect.has_value());
  EXPECT_TRUE(equals(*rect, CellRect{29, 29, 51, 51}));
}

TEST(PublishRectTest, ClampsToTheLowerLeftCorner)
{
  const auto rect = publish_rect(CellRect{0, 0, 0, 0}, 2, make_geometry(20, 20));
  ASSERT_TRUE(rect.has_value());
  EXPECT_TRUE(equals(*rect, CellRect{0, 0, 2, 2}));
}

TEST(PublishRectTest, ClampsToTheUpperRightCorner)
{
  const auto rect = publish_rect(CellRect{19, 19, 19, 19}, 2, make_geometry(20, 20));
  ASSERT_TRUE(rect.has_value());
  EXPECT_TRUE(equals(*rect, CellRect{17, 17, 19, 19}));
}

TEST(PublishRectTest, AWholeMapChangeRegionStaysTheWholeMap)
{
  const auto rect = publish_rect(CellRect{0, 0, 19, 19}, 5, make_geometry(20, 20));
  ASSERT_TRUE(rect.has_value());
  EXPECT_TRUE(equals(*rect, CellRect{0, 0, 19, 19}));
}

TEST(PublishRectTest, AOneCellMapCannotGrowOutOfItself)
{
  const auto rect = publish_rect(CellRect{0, 0, 0, 0}, 3, make_geometry(1, 1));
  ASSERT_TRUE(rect.has_value());
  EXPECT_TRUE(equals(*rect, CellRect{0, 0, 0, 0}));
}

TEST(PublishRectTest, ASaturatedMarginClampsInsteadOfOverflowing)
{
  const auto rect =
    publish_rect(CellRect{10, 10, 10, 10}, std::numeric_limits<int>::max(), make_geometry(20, 20));
  ASSERT_TRUE(rect.has_value());
  EXPECT_TRUE(equals(*rect, CellRect{0, 0, 19, 19}));
}

TEST(PublishRectTest, KeepsTheTwoAxesApartOnANonSquareMap)
{
  const auto rect = publish_rect(CellRect{2, 1, 2, 1}, 1, make_geometry(4, 3));
  ASSERT_TRUE(rect.has_value());
  EXPECT_TRUE(equals(*rect, CellRect{1, 0, 3, 2}));
}

TEST(PublishRectTest, HasNothingToPublishForADegenerateGeometry)
{
  EXPECT_FALSE(publish_rect(CellRect{0, 0, 0, 0}, 1, MapGeometry{}).has_value());
  EXPECT_FALSE(publish_rect(CellRect{0, 0, 0, 0}, 1, make_geometry(0, 5)).has_value());
}

}  // namespace
