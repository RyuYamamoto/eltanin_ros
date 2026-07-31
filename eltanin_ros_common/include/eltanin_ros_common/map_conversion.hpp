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

#ifndef ELTANIN_ROS_COMMON__MAP_CONVERSION_HPP_
#define ELTANIN_ROS_COMMON__MAP_CONVERSION_HPP_

#include "eltanin_ros_common/conversion_result.hpp"
#include "eltanin_ros_common/cost_conversion.hpp"

#include <builtin_interfaces/msg/time.hpp>
#include <eltanin/map/grid_map.hpp>

#include <eltanin_msgs/msg/costmap.hpp>
#include <eltanin_msgs/msg/costmap_update.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <cstddef>
#include <string>

namespace eltanin_ros_common
{

/// Upper bound on what a message is allowed to make this process allocate.
struct MapLimits
{
  std::size_t max_cells{64000000};
};

/// A static map; rejected unless its origin is a pure translation, which is all MapGeometry has.
ConversionResult<eltanin::map::Costmap> to_costmap(
  const nav_msgs::msg::OccupancyGrid & msg, const OccupancyThresholds & thresholds,
  const MapLimits & limits = {});

/// The internal topic; raw cost values, and the only direction that loses nothing.
ConversionResult<eltanin::map::Costmap> to_costmap(
  const eltanin_msgs::msg::Costmap & msg, const MapLimits & limits = {});

/// frame_id and stamp are arguments so that no frame name can be written into this layer.
ConversionResult<eltanin_msgs::msg::Costmap> to_costmap_msg(
  const eltanin::map::Costmap & costmap, const std::string & frame_id,
  const builtin_interfaces::msg::Time & stamp);

/// One rectangular patch in raw costs; the rectangle is checked against the map, not trusted.
ConversionResult<eltanin_msgs::msg::CostmapUpdate> to_costmap_update_msg(
  const eltanin::map::Costmap & costmap, const eltanin::map::CellRect & rect,
  const std::string & frame_id, const builtin_interfaces::msg::Time & stamp);

/// For visualization only; the cost range is compressed and does not survive a round trip.
ConversionResult<nav_msgs::msg::OccupancyGrid> to_occupancy_grid(
  const eltanin::map::Costmap & costmap, const std::string & frame_id,
  const builtin_interfaces::msg::Time & stamp);

}  // namespace eltanin_ros_common

#endif  // ELTANIN_ROS_COMMON__MAP_CONVERSION_HPP_
