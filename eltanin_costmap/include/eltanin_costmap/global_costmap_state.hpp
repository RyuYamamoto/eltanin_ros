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

#ifndef ELTANIN_COSTMAP__GLOBAL_COSTMAP_STATE_HPP_
#define ELTANIN_COSTMAP__GLOBAL_COSTMAP_STATE_HPP_

#include "eltanin_costmap/observation_store.hpp"

#include <eltanin/map/cost_model.hpp>
#include <eltanin/map/grid_map.hpp>
#include <eltanin/map/layered_costmap.hpp>
#include <eltanin/map/layers/obstacle_layer.hpp>
#include <eltanin_ros_common/conversion_result.hpp>
#include <eltanin_ros_common/cost_conversion.hpp>
#include <eltanin_ros_common/map_conversion.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>

#include <cstddef>
#include <optional>
#include <string>

namespace eltanin_costmap
{

/// The only gate into a GlobalCostmapState: the configured frame, then every check on a static map.
eltanin_ros_common::ConversionResult<eltanin::map::Costmap> accept_static_map(
  const nav_msgs::msg::OccupancyGrid & msg, const std::string & expected_frame_id,
  const eltanin_ros_common::OccupancyThresholds & thresholds,
  const eltanin_ros_common::MapLimits & limits = {});

/// Everything one static map fixes; a new map replaces the whole object rather than mutating it.
class GlobalCostmapState
{
public:
  /// `static_map` must come from accept_static_map(), which discharges eltanin's asserts (R-18).
  GlobalCostmapState(
    eltanin::map::Costmap static_map, const eltanin::map::InflationCostModel & inflation,
    bool inflate_unknown);

  struct UpdateOutcome
  {
    /// The change region grown by r; nullopt when nothing changed since the previous update.
    std::optional<eltanin::map::CellRect> publish_rect;

    std::size_t observation_count{0};
  };

  /// Regenerates the whole master; called from the /map callback and ~/update, never a timer (C-3).
  UpdateOutcome update();

  ObservationStore::AbsorbOutcome absorb(const eltanin::map::Costmap & local_map);

  /// Drops the observations without updating or publishing, so the two triggers stay the only ones.
  void clear_observations() noexcept;

  const eltanin::map::Costmap & costmap() const noexcept;

  const eltanin::map::MapGeometry & geometry() const noexcept;

  std::size_t observation_count() const noexcept;

  int inflation_cells() const noexcept;

private:
  /// Keeps the static map's geometry for life; nothing here calls set_origin() or center_on().
  eltanin::map::LayeredCostmap costmap_;

  /// A reference, so it cannot outlive the LayeredCostmap that owns the layer (A-P6-5).
  eltanin::map::ObstacleLayer & obstacle_layer_;

  int inflation_cells_{0};

  ObservationStore observations_;
};

}  // namespace eltanin_costmap

#endif  // ELTANIN_COSTMAP__GLOBAL_COSTMAP_STATE_HPP_
