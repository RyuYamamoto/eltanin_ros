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

#include "eltanin_ros_common/diagnostic_conversion.hpp"

#include <cmath>
#include <sstream>
#include <utility>

namespace eltanin_ros_common
{

namespace
{

diagnostic_msgs::msg::KeyValue key_value(const std::string & key, std::string value)
{
  diagnostic_msgs::msg::KeyValue pair;
  pair.key = key;
  pair.value = std::move(value);
  return pair;
}

}  // namespace

std::string to_diagnostic_value(double value)
{
  if (std::isnan(value)) {
    return "nan";
  }
  if (std::isinf(value)) {
    return value > 0.0 ? "inf" : "-inf";
  }
  std::ostringstream stream;
  // Six significant digits is plenty for a metre or a second and keeps the line readable.
  stream << value;
  return stream.str();
}

DiagnosticStatusBuilder::DiagnosticStatusBuilder(
  std::string name, std::uint8_t level, std::string message)
{
  status_.name = std::move(name);
  status_.level = level;
  status_.message = std::move(message);
}

DiagnosticStatusBuilder & DiagnosticStatusBuilder::add(const std::string & key, double value)
{
  status_.values.push_back(key_value(key, to_diagnostic_value(value)));
  return *this;
}

DiagnosticStatusBuilder & DiagnosticStatusBuilder::add(const std::string & key, int value)
{
  status_.values.push_back(key_value(key, std::to_string(value)));
  return *this;
}

DiagnosticStatusBuilder & DiagnosticStatusBuilder::add(const std::string & key, std::size_t value)
{
  status_.values.push_back(key_value(key, std::to_string(value)));
  return *this;
}

DiagnosticStatusBuilder & DiagnosticStatusBuilder::add(const std::string & key, bool value)
{
  status_.values.push_back(key_value(key, value ? "true" : "false"));
  return *this;
}

DiagnosticStatusBuilder & DiagnosticStatusBuilder::add(
  const std::string & key, const std::string & value)
{
  status_.values.push_back(key_value(key, value));
  return *this;
}

DiagnosticStatusBuilder & DiagnosticStatusBuilder::add(const std::string & key, const char * value)
{
  status_.values.push_back(key_value(key, std::string(value)));
  return *this;
}

}  // namespace eltanin_ros_common
