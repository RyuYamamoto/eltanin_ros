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

#ifndef ELTANIN_CONTROLLER__PATH_INPUT_HPP_
#define ELTANIN_CONTROLLER__PATH_INPUT_HPP_

#include <eltanin/core/path.hpp>
#include <rclcpp/time.hpp>

#include <memory>
#include <optional>
#include <string>

namespace eltanin_controller
{

/// One accepted path; the subscription converts once and the timer copies a pointer, not poses.
struct PathSnapshot
{
  std::shared_ptr<const eltanin::Path> path;
  /// The message header stamp, which is what staleness is measured against.
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
};

/// Holds the last accepted path and the last rejection behind a deadline that may be absent.
class PathInput
{
public:
  enum class State { NeverReceived, Rejected, Stale, Available };

  struct Reading
  {
    State state{State::NeverReceived};
    PathSnapshot snapshot;
    /// now - stamp [s], negative for a stamp in the future; for the diagnostic, not for the check.
    double elapsed_seconds{0.0};
  };

  /// nullopt disables the deadline entirely; a value is expected to be positive and finite.
  explicit PathInput(std::optional<double> timeout_seconds) noexcept;

  /// Replaces the snapshot and clears the rejection record.
  void accept(PathSnapshot snapshot);

  /// Records why the last message was dropped; a path already held keeps being followed.
  void reject(std::string detail);

  /// The snapshot comes back by value so that no caller computes while holding the node's lock.
  Reading read(const rclcpp::Time & now) const;

  const std::string & rejection_detail() const noexcept;

private:
  std::optional<double> timeout_seconds_;
  std::optional<PathSnapshot> snapshot_;
  std::string rejection_;
};

}  // namespace eltanin_controller

#endif  // ELTANIN_CONTROLLER__PATH_INPUT_HPP_
