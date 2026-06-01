// Copyright 2026 Siddhartha Dubey
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
// limitations under the License.

#include "bopt_box_filter/box_filter.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

namespace bopt_box_filter
{

void apply_box_filter(sensor_msgs::msg::LaserScan & scan, const Box & box)
{
  const float inf = std::numeric_limits<float>::infinity();
  const size_t n = scan.ranges.size();
  for (size_t i = 0; i < n; ++i) {
    const float r = scan.ranges[i];
    if (!std::isfinite(r) || r < scan.range_min || r > scan.range_max) {
      continue;
    }
    const float angle = scan.angle_min +
      static_cast<float>(i) * scan.angle_increment;
    const double x = static_cast<double>(r) * std::cos(angle);
    const double y = static_cast<double>(r) * std::sin(angle);
    if (x >= box.min_x && x <= box.max_x &&
      y >= box.min_y && y <= box.max_y)
    {
      scan.ranges[i] = inf;
    }
  }
}

bool update_triggered_state(
  bool current_state,
  const std_msgs::msg::Int8MultiArray & msg)
{
  if (msg.data.size() < 4) {
    return current_state;
  }
  const int8_t value = msg.data[3];
  if (value == 1) {
    return true;
  }
  if (value == 0) {
    return false;
  }
  return current_state;
}

Box box_from_array(const std::string & name, const std::vector<double> & arr)
{
  if (arr.size() != 4) {
    throw std::invalid_argument(
            "parameter '" + name +
            "' must have exactly 4 elements [min_x, max_x, min_y, max_y], got " +
            std::to_string(
              arr.size()));
  }
  Box box{arr[0], arr[1], arr[2], arr[3]};
  if (box.min_x >= box.max_x) {
    throw std::invalid_argument(
            "parameter '" + name + "' has min_x (" + std::to_string(box.min_x) +
            ") >= max_x (" + std::to_string(box.max_x) + ")");
  }
  if (box.min_y >= box.max_y) {
    throw std::invalid_argument(
            "parameter '" + name + "' has min_y (" + std::to_string(box.min_y) +
            ") >= max_y (" + std::to_string(box.max_y) + ")");
  }
  return box;
}

}  // namespace bopt_box_filter
