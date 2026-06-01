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

#ifndef BOPT_BOX_FILTER__BOX_FILTER_HPP_
#define BOPT_BOX_FILTER__BOX_FILTER_HPP_

#include <stdexcept>
#include <string>
#include <vector>

#include "bopt_box_filter/box.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/int8_multi_array.hpp"

namespace bopt_box_filter
{

// Masks rays whose (x, y) endpoint (in the laser's own frame) falls
// inclusively inside box. Masked rays are set to +infinity.
// Rays already non-finite or outside [range_min, range_max] are untouched.
// Header and geometry fields are untouched.
void apply_box_filter(sensor_msgs::msg::LaserScan & scan, const Box & box);

// Returns the next triggered state given the current state and a feedback msg.
// - data[3] == 1 -> true
// - data[3] == 0 -> false
// - any other value, or data.size() < 4 -> current_state (unchanged)
//
// The caller is responsible for logging; this function is deterministic and
// side-effect free.
bool update_triggered_state(
  bool current_state,
  const std_msgs::msg::Int8MultiArray & msg);

// Converts a 4-element parameter array [min_x, max_x, min_y, max_y] into a Box.
// Throws std::invalid_argument with a message referencing `name` if:
// - arr.size() != 4
// - min_x >= max_x  or  min_y >= max_y
Box box_from_array(const std::string & name, const std::vector<double> & arr);

}  // namespace bopt_box_filter

#endif  // BOPT_BOX_FILTER__BOX_FILTER_HPP_
