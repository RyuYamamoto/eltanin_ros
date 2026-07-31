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

#ifndef ELTANIN_COSTMAP__UPDATE_REGION_HPP_
#define ELTANIN_COSTMAP__UPDATE_REGION_HPP_

#include <eltanin/map/map_geometry.hpp>

#include <optional>

namespace eltanin_costmap
{

/// The inflation radius in cells, by the ceil() and the saturation InflationLayer's own table uses.
int inflation_cells(double inflation_radius, double resolution) noexcept;

/// The cells that changed since the last update, as one bounding rectangle that never shrinks.
class ChangeRegion
{
public:
  void add(const eltanin::map::CellRect & rect) noexcept;

  void clear() noexcept;

  bool empty() const noexcept { return !rect_.has_value(); }

  /// nullopt while empty, so there is no spelling of "nothing changed" that reads as a rectangle.
  const std::optional<eltanin::map::CellRect> & rect() const noexcept { return rect_; }

private:
  std::optional<eltanin::map::CellRect> rect_;
};

/// The change region grown by `cells` on each side and clamped to the map; nullopt in, nullopt out.
std::optional<eltanin::map::CellRect> publish_rect(
  const std::optional<eltanin::map::CellRect> & change_region, int cells,
  const eltanin::map::MapGeometry & geometry) noexcept;

}  // namespace eltanin_costmap

#endif  // ELTANIN_COSTMAP__UPDATE_REGION_HPP_
