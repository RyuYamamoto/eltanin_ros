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

#include "eltanin_controller/path_input.hpp"

#include <cmath>
#include <utility>

namespace eltanin_controller
{

namespace
{

/// Raw nanoseconds rather than rclcpp::Time, whose comparison throws on a clock_type mismatch.
double seconds_between(const rclcpp::Time & now, const rclcpp::Time & stamp)
{
  return (static_cast<double>(now.nanoseconds()) - static_cast<double>(stamp.nanoseconds())) * 1e-9;
}

}  // namespace

PathInput::PathInput(std::optional<double> timeout_seconds) noexcept
: timeout_seconds_(timeout_seconds)
{
}

void PathInput::accept(PathSnapshot snapshot)
{
  snapshot_ = std::move(snapshot);
  rejection_.clear();
}

void PathInput::reject(std::string detail)
{
  rejection_ = std::move(detail);
}

void PathInput::clear()
{
  snapshot_.reset();
  rejection_.clear();
}

PathInput::Reading PathInput::read(const rclcpp::Time & now) const
{
  Reading reading;
  if (!snapshot_.has_value()) {
    reading.state = rejection_.empty() ? State::NeverReceived : State::Rejected;
    return reading;
  }

  reading.snapshot = *snapshot_;
  reading.elapsed_seconds = seconds_between(now, snapshot_->stamp);
  // The deadline is checked on both sides: a stamp from the future is no fresher than an old one.
  if (timeout_seconds_.has_value() && std::abs(reading.elapsed_seconds) > *timeout_seconds_) {
    reading.state = State::Stale;
    return reading;
  }
  reading.state = State::Available;
  return reading;
}

const std::string & PathInput::rejection_detail() const noexcept
{
  return rejection_;
}

}  // namespace eltanin_controller
