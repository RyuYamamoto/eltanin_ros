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

#include "eltanin_ros_common/scan_conversion.hpp"

#include <cmath>
#include <string>
#include <utility>

namespace eltanin_ros_common
{

namespace
{

std::string reject(const sensor_msgs::msg::LaserScan & msg, const std::string & violation)
{
  return "eltanin_ros_common: rejected LaserScan (frame_id='" + msg.header.frame_id + "', " +
         std::to_string(msg.ranges.size()) + " beams): " + violation;
}

}  // namespace

ConversionResult<StampedScan> to_scan(const sensor_msgs::msg::LaserScan & msg)
{
  if (!std::isfinite(msg.angle_min) || !std::isfinite(msg.angle_increment)) {
    return ConversionResult<StampedScan>::failure(reject(
      msg, "angle_min " + std::to_string(msg.angle_min) + " and angle_increment " +
             std::to_string(msg.angle_increment) + " must both be finite"));
  }
  if (!std::isfinite(msg.range_min)) {
    return ConversionResult<StampedScan>::failure(
      reject(msg, "range_min " + std::to_string(msg.range_min) + " must be finite"));
  }
  if (std::isnan(msg.range_max)) {
    return ConversionResult<StampedScan>::failure(
      reject(msg, "range_max must be finite or +inf, not NaN"));
  }
  if (msg.range_min < 0.0F) {
    return ConversionResult<StampedScan>::failure(
      reject(msg, "range_min " + std::to_string(msg.range_min) + " must not be negative"));
  }
  if (msg.range_min > msg.range_max) {
    return ConversionResult<StampedScan>::failure(reject(
      msg, "range_min " + std::to_string(msg.range_min) + " is above range_max " +
             std::to_string(msg.range_max)));
  }

  StampedScan stamped;
  stamped.stamp = msg.header.stamp;
  stamped.frame_id = msg.header.frame_id;
  stamped.scan.angle_min = msg.angle_min;
  stamped.scan.angle_increment = msg.angle_increment;
  stamped.scan.range_min = msg.range_min;
  stamped.scan.range_max = msg.range_max;
  stamped.scan.ranges = msg.ranges;
  return ConversionResult<StampedScan>::success(std::move(stamped));
}

}  // namespace eltanin_ros_common
