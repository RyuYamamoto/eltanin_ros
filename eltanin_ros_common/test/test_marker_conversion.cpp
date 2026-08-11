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

#include "eltanin_ros_common/marker_conversion.hpp"
#include "test/conversion_test_helpers.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <limits>
#include <numbers>
#include <string>
#include <vector>

namespace
{

using eltanin_ros_common::FootprintMarkerStyle;
using eltanin_ros_common::to_footprint_markers;
using eltanin_ros_common::to_predicted_footprint_markers;
using eltanin_ros_common::test::contains;
using eltanin_ros_common::test::is_one_line;
using eltanin_ros_common::test::names_the_package_once;
using visualization_msgs::msg::Marker;

builtin_interfaces::msg::Time make_stamp(std::int32_t sec = 3, std::uint32_t nanosec = 4)
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = sec;
  stamp.nanosec = nanosec;
  return stamp;
}

/// A rectangle that is not symmetric in x, so a rotation cannot look like the identity.
eltanin::Polygon2D make_footprint()
{
  return eltanin::Polygon2D{
    Eigen::Vector2d{-0.15, -0.12}, Eigen::Vector2d{0.237, -0.12}, Eigen::Vector2d{0.237, 0.12},
    Eigen::Vector2d{-0.15, 0.12}};
}

eltanin::Path make_path(std::size_t poses, double yaw = 0.0)
{
  std::vector<eltanin::Pose2D> values;
  for (std::size_t at = 0; at < poses; ++at) {
    values.push_back(eltanin::Pose2D{Eigen::Vector2d{static_cast<double>(at), 0.0}, yaw});
  }
  return eltanin::Path(values);
}

/// Markers other than the leading DELETEALL.
std::vector<Marker> outlines(const visualization_msgs::msg::MarkerArray & markers)
{
  return std::vector<Marker>(markers.markers.begin() + 1, markers.markers.end());
}

TEST(ToFootprintMarkersTest, AlwaysLeadsWithDeleteAllSoAShorterPathLeavesNothingBehind)
{
  const auto markers = to_footprint_markers(make_path(4), make_footprint(), 1, "map", make_stamp());
  ASSERT_TRUE(markers.ok()) << markers.error();
  ASSERT_FALSE(markers.value().markers.empty());
  EXPECT_EQ(markers.value().markers.front().action, Marker::DELETEALL);
  EXPECT_EQ(markers.value().markers.front().ns, "footprint");
}

TEST(ToFootprintMarkersTest, PlacesOneClosedOutlinePerSampledPose)
{
  const auto markers = to_footprint_markers(make_path(4), make_footprint(), 1, "map", make_stamp());
  ASSERT_TRUE(markers.ok()) << markers.error();
  const std::vector<Marker> drawn = outlines(markers.value());
  ASSERT_EQ(drawn.size(), 4u);
  for (const Marker & marker : drawn) {
    EXPECT_EQ(marker.type, Marker::LINE_STRIP);
    EXPECT_EQ(marker.action, Marker::ADD);
    EXPECT_EQ(marker.header.frame_id, "map");
    EXPECT_EQ(marker.header.stamp.sec, 3);
    // Four vertices plus the repeat that closes the strip.
    ASSERT_EQ(marker.points.size(), 5u);
    EXPECT_DOUBLE_EQ(marker.points.front().x, marker.points.back().x);
    EXPECT_DOUBLE_EQ(marker.points.front().y, marker.points.back().y);
    // The points are already in frame_id, so the marker pose must be the identity.
    EXPECT_DOUBLE_EQ(marker.pose.orientation.w, 1.0);
    EXPECT_DOUBLE_EQ(marker.pose.position.x, 0.0);
  }
}

TEST(ToFootprintMarkersTest, GivesEveryOutlineItsOwnId)
{
  const auto markers = to_footprint_markers(make_path(3), make_footprint(), 1, "map", make_stamp());
  ASSERT_TRUE(markers.ok()) << markers.error();
  const std::vector<Marker> drawn = outlines(markers.value());
  ASSERT_EQ(drawn.size(), 3u);
  EXPECT_EQ(drawn[0].id, 0);
  EXPECT_EQ(drawn[1].id, 1);
  EXPECT_EQ(drawn[2].id, 2);
}

TEST(ToFootprintMarkersTest, TheStrideSkipsPosesButNeverTheLastOne)
{
  const auto markers =
    to_footprint_markers(make_path(10), make_footprint(), 4, "map", make_stamp());
  ASSERT_TRUE(markers.ok()) << markers.error();
  const std::vector<Marker> drawn = outlines(markers.value());
  // 0, 4, 8 and then the last pose at 9.
  ASSERT_EQ(drawn.size(), 4u);
  EXPECT_NEAR(drawn[0].points[0].x, -0.15, 1e-9);
  EXPECT_NEAR(drawn[3].points[0].x, 9.0 - 0.15, 1e-9);
}

TEST(ToFootprintMarkersTest, ASinglePosePathDrawsExactlyOneOutline)
{
  const auto markers = to_footprint_markers(make_path(1), make_footprint(), 5, "map", make_stamp());
  ASSERT_TRUE(markers.ok()) << markers.error();
  EXPECT_EQ(outlines(markers.value()).size(), 1u);
}

TEST(ToFootprintMarkersTest, PlacesTheFootprintAtThePoseAndTurnsItWithTheYaw)
{
  std::vector<eltanin::Pose2D> poses{
    eltanin::Pose2D{Eigen::Vector2d{2.0, 3.0}, 0.5 * std::numbers::pi}};
  const auto markers =
    to_footprint_markers(eltanin::Path(poses), make_footprint(), 1, "map", make_stamp());
  ASSERT_TRUE(markers.ok()) << markers.error();
  const std::vector<Marker> drawn = outlines(markers.value());
  ASSERT_EQ(drawn.size(), 1u);
  // (-0.15, -0.12) turned by +90 degrees is (0.12, -0.15), then moved to (2, 3).
  EXPECT_NEAR(drawn[0].points[0].x, 2.0 + 0.12, 1e-9);
  EXPECT_NEAR(drawn[0].points[0].y, 3.0 - 0.15, 1e-9);
}

TEST(ToFootprintMarkersTest, AnEmptyPathIsNotAnErrorButClearsWhatWasDrawn)
{
  const auto markers =
    to_footprint_markers(eltanin::Path{}, make_footprint(), 1, "map", make_stamp());
  ASSERT_TRUE(markers.ok()) << markers.error();
  ASSERT_EQ(markers.value().markers.size(), 1u);
  EXPECT_EQ(markers.value().markers.front().action, Marker::DELETEALL);
}

TEST(ToFootprintMarkersTest, RejectsAFootprintThatIsNotAPolygon)
{
  const eltanin::Polygon2D two_points{Eigen::Vector2d{0.0, 0.0}, Eigen::Vector2d{1.0, 0.0}};
  const auto markers = to_footprint_markers(make_path(2), two_points, 1, "map", make_stamp());
  EXPECT_FALSE(markers.ok());
  EXPECT_TRUE(contains(markers.error(), "at least 3"));
  EXPECT_TRUE(is_one_line(markers.error()));
  EXPECT_TRUE(names_the_package_once(markers.error()));
}

TEST(ToFootprintMarkersTest, RejectsAStrideBelowOne)
{
  const auto markers = to_footprint_markers(make_path(2), make_footprint(), 0, "map", make_stamp());
  EXPECT_FALSE(markers.ok());
  EXPECT_TRUE(contains(markers.error(), "at least 1"));
  EXPECT_TRUE(is_one_line(markers.error()));
}

TEST(ToFootprintMarkersTest, RejectsANonFinitePoseOrVertex)
{
  const double not_a_number = std::numeric_limits<double>::quiet_NaN();
  std::vector<eltanin::Pose2D> poses{eltanin::Pose2D{Eigen::Vector2d{not_a_number, 0.0}, 0.0}};
  const auto bad_pose =
    to_footprint_markers(eltanin::Path(poses), make_footprint(), 1, "map", make_stamp());
  EXPECT_FALSE(bad_pose.ok());
  EXPECT_TRUE(contains(bad_pose.error(), "not finite"));

  eltanin::Polygon2D bad_shape = make_footprint();
  bad_shape.push_back(Eigen::Vector2d{not_a_number, 0.0});
  const auto bad_vertex = to_footprint_markers(make_path(2), bad_shape, 1, "map", make_stamp());
  EXPECT_FALSE(bad_vertex.ok());
  EXPECT_TRUE(contains(bad_vertex.error(), "not finite"));
}

TEST(ToFootprintMarkersTest, RejectsALineWidthThatCannotBeDrawn)
{
  FootprintMarkerStyle style;
  style.line_width = 0.0;
  const auto markers = to_footprint_markers(
    make_path(2), make_footprint(), 1, "map", make_stamp(), "footprint", style);
  EXPECT_FALSE(markers.ok());
  EXPECT_TRUE(contains(markers.error(), "line_width"));
}

TEST(ToFootprintMarkersTest, TheNamespaceReachesEveryMarkerIncludingTheDeleteAll)
{
  const auto markers =
    to_footprint_markers(make_path(2), make_footprint(), 1, "map", make_stamp(), "plan_footprint");
  ASSERT_TRUE(markers.ok()) << markers.error();
  for (const Marker & marker : markers.value().markers) {
    EXPECT_EQ(marker.ns, "plan_footprint");
  }
}

std::vector<std_msgs::msg::ColorRGBA> colors_for(std::size_t poses)
{
  std::vector<std_msgs::msg::ColorRGBA> colors(poses);
  for (std::size_t index = 0; index < poses; ++index) {
    colors[index].g = static_cast<float>(index) / static_cast<float>(poses);
    colors[index].a = 1.0F;
  }
  return colors;
}

TEST(ToPredictedFootprintMarkersTest, DrawsOneOutlinePerPoseInItsOwnColour)
{
  const auto markers = to_predicted_footprint_markers(
    make_path(4), make_footprint(), colors_for(4), "map", make_stamp(), false);
  ASSERT_TRUE(markers.ok()) << markers.error();

  // One DELETEALL for the footprints, four outlines, and one DELETEALL for the absent contact.
  ASSERT_EQ(markers.value().markers.size(), 6u);
  EXPECT_EQ(markers.value().markers.front().action, Marker::DELETEALL);
  int outlines = 0;
  for (const Marker & marker : markers.value().markers) {
    if (marker.type == Marker::LINE_STRIP && marker.action == Marker::ADD) {
      ++outlines;
    }
  }
  EXPECT_EQ(outlines, 4);
  EXPECT_FLOAT_EQ(markers.value().markers[4].color.g, 0.75F);
}

TEST(ToPredictedFootprintMarkersTest, MarksTheContactPointOnlyWhenAsked)
{
  const auto with_contact = to_predicted_footprint_markers(
    make_path(3), make_footprint(), colors_for(3), "map", make_stamp(), true);
  ASSERT_TRUE(with_contact.ok());
  int spheres = 0;
  for (const Marker & marker : with_contact.value().markers) {
    if (marker.type == Marker::SPHERE) {
      ++spheres;
    }
  }
  EXPECT_EQ(spheres, 1);

  const auto without = to_predicted_footprint_markers(
    make_path(3), make_footprint(), colors_for(3), "map", make_stamp(), false);
  ASSERT_TRUE(without.ok());
  for (const Marker & marker : without.value().markers) {
    EXPECT_NE(marker.type, Marker::SPHERE);
  }
}

TEST(ToPredictedFootprintMarkersTest, RejectsAColourCountThatDoesNotMatchThePoses)
{
  const auto markers = to_predicted_footprint_markers(
    make_path(4), make_footprint(), colors_for(3), "map", make_stamp(), false);
  EXPECT_FALSE(markers.ok());
  EXPECT_TRUE(contains(markers.error(), "one per pose"));
}

TEST(ToPredictedFootprintMarkersTest, AnEmptyPathClearsWithoutDrawing)
{
  const auto markers = to_predicted_footprint_markers(
    eltanin::Path{}, make_footprint(), {}, "map", make_stamp(), false);
  ASSERT_TRUE(markers.ok()) << markers.error();
  ASSERT_EQ(markers.value().markers.size(), 1u);
  EXPECT_EQ(markers.value().markers.front().action, Marker::DELETEALL);
}

}  // namespace
