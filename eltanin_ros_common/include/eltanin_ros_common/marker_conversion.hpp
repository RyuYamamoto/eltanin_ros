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

#ifndef ELTANIN_ROS_COMMON__MARKER_CONVERSION_HPP_
#define ELTANIN_ROS_COMMON__MARKER_CONVERSION_HPP_

#include "eltanin_ros_common/conversion_result.hpp"

#include <builtin_interfaces/msg/time.hpp>
#include <eltanin/core/path.hpp>
#include <eltanin/core/polygon.hpp>

#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <string>
#include <vector>

namespace eltanin_ros_common
{

/// How the outline is drawn; visualization only, so none of it affects a decision.
struct FootprintMarkerStyle
{
  double line_width{0.02};
  /// Blue, not cyan: the costmap colour scheme spends cyan on cost 99, which the outline sits on.
  float red{0.0F};
  float green{0.2F};
  float blue{1.0F};
  float alpha{0.8F};
};

/// The footprint placed at every stride-th pose of a path, plus the last one, as closed outlines.
ConversionResult<visualization_msgs::msg::MarkerArray> to_footprint_markers(
  const eltanin::Path & path, const eltanin::Polygon2D & footprint, int stride,
  const std::string & frame_id, const builtin_interfaces::msg::Time & stamp,
  const std::string & ns = "footprint", const FootprintMarkerStyle & style = {});

/// The footprint at every pose, each in the colour the caller passes, plus a sphere at the last
/// pose when mark_contact is set. What the colours mean is the caller's to decide; this only draws
/// them. colors must hold one entry per pose. line_width [m] is the outline thickness.
ConversionResult<visualization_msgs::msg::MarkerArray> to_predicted_footprint_markers(
  const eltanin::Path & path, const eltanin::Polygon2D & footprint,
  const std::vector<std_msgs::msg::ColorRGBA> & colors, const std::string & frame_id,
  const builtin_interfaces::msg::Time & stamp, bool mark_contact,
  const std::string & ns = "predicted_footprint", double line_width = 0.02);

}  // namespace eltanin_ros_common

#endif  // ELTANIN_ROS_COMMON__MARKER_CONVERSION_HPP_
