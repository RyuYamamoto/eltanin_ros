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

#include <cmath>
#include <cstddef>
#include <optional>
#include <span>
#include <utility>

namespace eltanin_costmap
{

namespace
{

/// /map narrows the resolution to float32, so a difference this small is narrowing, not real.
constexpr double RESOLUTION_TOLERANCE = 1e-9;

}  // namespace

ObservationStore::ObservationStore(eltanin::map::Costmap static_map)
: static_map_(std::move(static_map))
{
}

ObservationStore::AbsorbOutcome ObservationStore::absorb(const eltanin::map::Costmap & local_map)
{
  const eltanin::map::MapGeometry & local = local_map.geometry();
  const eltanin::map::MapGeometry & global = static_map_.geometry();

  AbsorbOutcome outcome;
  // Discharges the in_bounds and the min <= max preconditions of everything below (A-5 / R-18).
  if (
    local.size_x() <= 0 || local.size_y() <= 0 || !std::isfinite(local.resolution()) ||
    local.resolution() <= 0.0 || local_map.data().size() != local.cell_count()) {
    outcome.overlaps = false;
    return outcome;
  }
  outcome.resolution_differs =
    std::abs(local.resolution() - global.resolution()) > RESOLUTION_TOLERANCE;

  const std::optional<eltanin::map::CellRect> window = global.world_rect_to_cells(
    local.map_to_world(0, 0), local.map_to_world(local.size_x() - 1, local.size_y() - 1));
  if (!window.has_value()) {
    outcome.overlaps = false;
    return outcome;
  }
  // Every window received enters the change region, not only the cells that turned out to be new.
  change_.add(*window);

  // One iterator over data() rather than operator(): its length is cell_count(), checked above.
  auto cell = local_map.data().begin();
  for (int my = 0; my < local.size_y(); ++my) {
    for (int mx = 0; mx < local.size_x(); ++mx, ++cell) {
      if (*cell != eltanin::map::LETHAL_OBSTACLE) {
        continue;
      }
      const std::optional<eltanin::map::MapIndex> at =
        global.world_to_map(local.map_to_world(mx, my));
      if (!at.has_value()) {
        continue;
      }
      if (static_map_.get(at->x, at->y) == eltanin::map::LETHAL_OBSTACLE) {
        continue;
      }
      if (!seen_.insert(global.index(at->x, at->y)).second) {
        continue;
      }
      // The global cell centre, not the local one: the ObstacleLayer maps it back to this cell.
      points_.push_back(global.map_to_world(at->x, at->y));
      stored_.add(eltanin::map::CellRect{at->x, at->y, at->x, at->y});
      ++outcome.discovered;
    }
  }
  return outcome;
}

std::span<const Eigen::Vector2d> ObservationStore::points() const noexcept
{
  return std::span<const Eigen::Vector2d>(points_.data(), points_.size());
}

std::size_t ObservationStore::size() const noexcept
{
  return points_.size();
}

std::optional<eltanin::map::CellRect> ObservationStore::take_change_region() noexcept
{
  const std::optional<eltanin::map::CellRect> region = change_.rect();
  change_.clear();
  return region;
}

void ObservationStore::clear() noexcept
{
  // A consumer that only follows patches would otherwise keep the obstacles we just dropped.
  if (!stored_.empty()) {
    change_.add(*stored_.rect());
  }
  points_.clear();
  seen_.clear();
  stored_.clear();
}

}  // namespace eltanin_costmap
