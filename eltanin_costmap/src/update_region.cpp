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

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>

namespace eltanin_costmap
{

int inflation_cells(double inflation_radius, double resolution) noexcept
{
  if (!std::isfinite(inflation_radius) || inflation_radius <= 0.0) {
    return 0;
  }
  if (!std::isfinite(resolution) || resolution <= 0.0) {
    return 0;
  }
  constexpr double MAX_RADIUS_IN_CELLS = static_cast<double>(std::numeric_limits<int>::max());
  const double radius_in_cells = std::ceil(inflation_radius / resolution);
  return static_cast<int>(std::min(radius_in_cells, MAX_RADIUS_IN_CELLS));
}

void ChangeRegion::add(const eltanin::map::CellRect & rect) noexcept
{
  if (!rect_.has_value()) {
    rect_ = rect;
    return;
  }
  rect_->min_x = std::min(rect_->min_x, rect.min_x);
  rect_->min_y = std::min(rect_->min_y, rect.min_y);
  rect_->max_x = std::max(rect_->max_x, rect.max_x);
  rect_->max_y = std::max(rect_->max_y, rect.max_y);
}

void ChangeRegion::clear() noexcept
{
  rect_.reset();
}

std::optional<eltanin::map::CellRect> publish_rect(
  const std::optional<eltanin::map::CellRect> & change_region, int cells,
  const eltanin::map::MapGeometry & geometry) noexcept
{
  if (!change_region.has_value()) {
    return std::nullopt;
  }
  const int size_x = geometry.size_x();
  const int size_y = geometry.size_y();
  if (size_x <= 0 || size_y <= 0) {
    return std::nullopt;
  }
  // Clamping the margin first is what keeps min - margin inside int when inflation_cells saturated.
  const int margin = std::clamp(cells, 0, std::max(size_x, size_y));
  const eltanin::map::CellRect & region = *change_region;
  return eltanin::map::CellRect{
    std::max(0, region.min_x - margin), std::max(0, region.min_y - margin),
    std::min(size_x - 1, region.max_x + margin), std::min(size_y - 1, region.max_y + margin)};
}

}  // namespace eltanin_costmap
