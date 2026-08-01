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

#ifndef ELTANIN_PLANNER__DIAGNOSTIC_HPP_
#define ELTANIN_PLANNER__DIAGNOSTIC_HPP_

#include <algorithm>
#include <string>
#include <string_view>

namespace eltanin_planner::diagnostic
{

inline constexpr std::string_view PREFIX = "eltanin_planner: ";

/// The shape of every line this package logs, so a diagnostic names its origin exactly once.
inline std::string line(std::string_view text)
{
  return std::string(PREFIX) + std::string(text);
}

/// The single shape every rejection has: who rejected it, what was rejected, and what was wrong.
inline std::string rejected(std::string_view subject, std::string_view violation)
{
  return line("rejected " + std::string(subject) + ": " + std::string(violation));
}

/// A diagnostic has to stay greppable as one line, whoever produced the text inside it.
inline std::string flatten(std::string text)
{
  std::replace(text.begin(), text.end(), '\n', ' ');
  std::replace(text.begin(), text.end(), '\r', ' ');
  return text;
}

}  // namespace eltanin_planner::diagnostic

#endif  // ELTANIN_PLANNER__DIAGNOSTIC_HPP_
