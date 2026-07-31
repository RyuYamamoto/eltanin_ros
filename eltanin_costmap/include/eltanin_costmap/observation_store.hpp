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

#ifndef ELTANIN_COSTMAP__OBSERVATION_STORE_HPP_
#define ELTANIN_COSTMAP__OBSERVATION_STORE_HPP_

#include "eltanin_costmap/update_region.hpp"

#include <eltanin/map/grid_map.hpp>

#include <cstddef>
#include <optional>
#include <span>
#include <unordered_set>
#include <vector>

namespace eltanin_costmap
{

/// The obstacles the static map does not have; within one static map this only ever grows (R-T6-2).
class ObservationStore
{
public:
  /// Takes the map the StaticLayer got; requires cell_count() > 0 and resolution() > 0 (R-18).
  explicit ObservationStore(eltanin::map::Costmap static_map);

  struct AbsorbOutcome
  {
    /// Cells occupied in this window that were not already known, static map or earlier window.
    std::size_t discovered{0};

    /// False when the window lies outside the static map, or is too degenerate to place at all.
    bool overlaps{true};

    /// The caller warns once and keeps the observations; refusing them loses obstacles (D-T6-8).
    bool resolution_differs{false};
  };

  /// Marks the LETHAL_OBSTACLE cells the static map lacks; never updates and never inflates (C-3).
  AbsorbOutcome absorb(const eltanin::map::Costmap & local_map);

  /// Discovery order, which is what makes the master costmap identical across runs (F-9).
  std::span<const Eigen::Vector2d> points() const noexcept;

  std::size_t size() const noexcept;

  /// Every window absorbed since the last take, plus what clear() dropped; moves the watermark.
  std::optional<eltanin::map::CellRect> take_change_region() noexcept;

  /// Drops every observation; the bbox of what went away enters the change region (D-P6-5).
  void clear() noexcept;

private:
  /// Owned, not referenced: a stored cell index only means a cell under this exact geometry.
  eltanin::map::Costmap static_map_;

  /// Global cell centres, so the ObstacleLayer rounds them back to the cells they came from.
  std::vector<Eigen::Vector2d> points_;

  /// Global linear indices; they keep their meaning only because this map's origin never moves.
  std::unordered_set<std::size_t> seen_;

  ChangeRegion change_;

  /// The bbox of everything in points_, kept so that clear() knows what it has to undo.
  ChangeRegion stored_;
};

}  // namespace eltanin_costmap

#endif  // ELTANIN_COSTMAP__OBSERVATION_STORE_HPP_
