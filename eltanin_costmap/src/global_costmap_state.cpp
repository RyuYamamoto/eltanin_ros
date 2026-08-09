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

#include <eltanin/map/cost_values.hpp>
#include <eltanin/map/layers/inflation_layer.hpp>
#include <eltanin/map/layers/static_layer.hpp>

#include <string>
#include <string_view>
#include <utility>

namespace eltanin_costmap
{

namespace
{

/// This package rejects the map, so it names itself; the rest of the line keeps the common shape.
constexpr std::string_view PREFIX = "eltanin_costmap: rejected ";

/// One line naming the frame, the resolution and the size, so it says which publisher was wrong.
std::string reject_grid(const nav_msgs::msg::OccupancyGrid & msg, const std::string & violation)
{
  return std::string(PREFIX) + "OccupancyGrid (frame_id='" + msg.header.frame_id +
         "', resolution=" + std::to_string(msg.info.resolution) + ", " +
         std::to_string(msg.info.width) + "x" + std::to_string(msg.info.height) + "): " + violation;
}

/// Registers the three layers in the fixed order and hands back the one the node has to feed.
eltanin::map::ObstacleLayer & add_layers(
  eltanin::map::LayeredCostmap & costmap, const eltanin::map::Costmap & static_map,
  const eltanin::map::InflationCostModel & inflation, bool inflate_unknown)
{
  costmap.add_layer<eltanin::map::StaticLayer>(static_map);
  eltanin::map::ObstacleLayer & obstacles = costmap.add_layer<eltanin::map::ObstacleLayer>();
  costmap.add_layer<eltanin::map::InflationLayer>(inflation, inflate_unknown);
  return obstacles;
}

}  // namespace

eltanin_ros_common::ConversionResult<eltanin::map::Costmap> accept_static_map(
  const nav_msgs::msg::OccupancyGrid & msg, const std::string & expected_frame_id,
  const eltanin_ros_common::OccupancyThresholds & thresholds,
  const eltanin_ros_common::MapLimits & limits)
{
  using Result = eltanin_ros_common::ConversionResult<eltanin::map::Costmap>;
  // The frame comes first because it is the one an operator can fix without touching the map.
  if (msg.header.frame_id != expected_frame_id) {
    return Result::failure(reject_grid(
      msg, "frame_id must be '" + expected_frame_id + "', which is what frames.map says"));
  }
  return eltanin_ros_common::to_costmap(msg, thresholds, limits);
}

GlobalCostmapState::GlobalCostmapState(
  eltanin::map::Costmap static_map, const eltanin::map::InflationCostModel & inflation,
  bool inflate_unknown)
// NO_INFORMATION, not FREE_SPACE: outside the static map and unobserved are the same thing (14.2).
: costmap_(static_map.geometry(), eltanin::map::NO_INFORMATION),
  obstacle_layer_(add_layers(costmap_, static_map, inflation, inflate_unknown)),
  inflation_cells_(eltanin_costmap::inflation_cells(
    inflation.distance_model().inflation_radius(), static_map.geometry().resolution())),
  // Moved last, because every initializer above still reads the geometry off it.
  observations_(std::move(static_map))
{
}

GlobalCostmapState::UpdateOutcome GlobalCostmapState::update()
{
  // The ObstacleLayer holds the points by copy, so they have to be handed over on every update.
  obstacle_layer_.set_points(observations_.points());
  costmap_.update();
  return UpdateOutcome{
    publish_rect(observations_.take_change_region(), inflation_cells_, costmap_.geometry()),
    observations_.size()};
}

ObservationStore::AbsorbOutcome GlobalCostmapState::absorb(const eltanin::map::Costmap & local_map)
{
  return observations_.absorb(local_map);
}

void GlobalCostmapState::clear_observations() noexcept
{
  observations_.clear();
}

const eltanin::map::Costmap & GlobalCostmapState::costmap() const noexcept
{
  return costmap_.costmap();
}

const eltanin::map::MapGeometry & GlobalCostmapState::geometry() const noexcept
{
  return costmap_.geometry();
}

std::size_t GlobalCostmapState::observation_count() const noexcept
{
  return observations_.size();
}

int GlobalCostmapState::inflation_cells() const noexcept
{
  return inflation_cells_;
}

}  // namespace eltanin_costmap
