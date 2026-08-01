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

#include "eltanin_planner/planner_parameters.hpp"

#include "src/diagnostic.hpp"

#include <cmath>
#include <string>

namespace eltanin_planner
{

namespace
{

using eltanin_ros_common::ConversionStatus;

/// The bound assert_smoother_params() states; it lives in a .cpp, so RelWithDebInfo drops it.
constexpr double SMOOTHER_CONVERGENCE_BOUND = 2.0;

ConversionStatus require_finite_non_negative(const char * key, double value)
{
  if (!std::isfinite(value)) {
    return ConversionStatus::failure(
      diagnostic::rejected(key, "is " + std::to_string(value) + ", which must be finite"));
  }
  if (value < 0.0) {
    return ConversionStatus::failure(
      diagnostic::rejected(key, "is " + std::to_string(value) + ", which must not be negative"));
  }
  return ConversionStatus::success();
}

ConversionStatus require_non_negative(const char * key, int value)
{
  if (value < 0) {
    return ConversionStatus::failure(
      diagnostic::rejected(key, "is " + std::to_string(value) + ", which must not be negative"));
  }
  return ConversionStatus::success();
}

}  // namespace

ConversionStatus validate(const PlannerParameters & parameters)
{
  const ConversionStatus radius =
    require_non_negative(KEY_START_SEARCH_RADIUS_CELLS, parameters.astar.start_search_radius_cells);
  if (!radius.ok()) {
    return radius;
  }
  const ConversionStatus weight_data =
    require_finite_non_negative(KEY_WEIGHT_DATA, parameters.smoother.weight_data);
  if (!weight_data.ok()) {
    return weight_data;
  }
  const ConversionStatus weight_smooth =
    require_finite_non_negative(KEY_WEIGHT_SMOOTH, parameters.smoother.weight_smooth);
  if (!weight_smooth.ok()) {
    return weight_smooth;
  }
  const ConversionStatus tolerance =
    require_finite_non_negative(KEY_SMOOTHER_TOLERANCE, parameters.smoother.tolerance);
  if (!tolerance.ok()) {
    return tolerance;
  }
  const ConversionStatus iterations =
    require_non_negative(KEY_SMOOTHER_MAX_ITERATIONS, parameters.smoother.max_iterations);
  if (!iterations.ok()) {
    return iterations;
  }
  const ConversionStatus timeout =
    require_finite_non_negative(KEY_TF_LOOKUP_TIMEOUT, parameters.tf_lookup_timeout);
  if (!timeout.ok()) {
    return timeout;
  }

  const double bound = parameters.smoother.weight_data + 4.0 * parameters.smoother.weight_smooth;
  if (bound >= SMOOTHER_CONVERGENCE_BOUND) {
    return ConversionStatus::failure(diagnostic::rejected(
      KEY_WEIGHT_SMOOTH,
      "weight_data " + std::to_string(parameters.smoother.weight_data) + " + 4 * weight_smooth " +
        std::to_string(parameters.smoother.weight_smooth) + " is " + std::to_string(bound) +
        ", which must be below " + std::to_string(SMOOTHER_CONVERGENCE_BOUND) +
        " or the smoother diverges"));
  }
  return ConversionStatus::success();
}

}  // namespace eltanin_planner
