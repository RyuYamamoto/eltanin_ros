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

#include "eltanin_controller/limiter_diagnostic.hpp"

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

namespace eltanin_controller
{

namespace
{

using Status = diagnostic_msgs::msg::DiagnosticStatus;

}  // namespace

const char * name_of(LimiterReason reason) noexcept
{
  switch (reason) {
    case LimiterReason::OutputDisabled:
      return "output_disabled";
    case LimiterReason::CommandRejected:
      return "command_rejected";
    case LimiterReason::CommandMissing:
      return "command_missing";
    case LimiterReason::CommandStale:
      return "command_stale";
    case LimiterReason::MapRejected:
      return "map_rejected";
    case LimiterReason::MapMissing:
      return "map_missing";
    case LimiterReason::MapStale:
      return "map_stale";
    case LimiterReason::NoTransform:
      return "no_transform";
    case LimiterReason::OutsideMap:
      return "outside_map";
    case LimiterReason::Limited:
      return "limited";
    case LimiterReason::None:
      return "none";
  }
  return "unknown";
}

std::uint8_t level_of(LimiterReason reason) noexcept
{
  switch (reason) {
    case LimiterReason::CommandRejected:
    case LimiterReason::MapRejected:
      return Status::ERROR;
    case LimiterReason::CommandMissing:
    case LimiterReason::CommandStale:
    case LimiterReason::MapMissing:
    case LimiterReason::MapStale:
      return Status::STALE;
    case LimiterReason::OutputDisabled:
    case LimiterReason::NoTransform:
    case LimiterReason::OutsideMap:
    case LimiterReason::Limited:
      return Status::WARN;
    case LimiterReason::None:
      return Status::OK;
  }
  return Status::WARN;
}

bool forces_zero_command(LimiterReason reason) noexcept
{
  return reason != LimiterReason::Limited && reason != LimiterReason::None;
}

}  // namespace eltanin_controller
