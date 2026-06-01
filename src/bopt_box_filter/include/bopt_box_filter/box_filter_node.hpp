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

#ifndef BOPT_BOX_FILTER__BOX_FILTER_NODE_HPP_
#define BOPT_BOX_FILTER__BOX_FILTER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/int8_multi_array.hpp"

#include "bopt_box_filter/box.hpp"

namespace bopt_box_filter
{

// This node assumes execution on a SingleThreadedExecutor. All three
// callbacks run serialized on one thread, so triggered_ needs no mutex.
// A MultiThreadedExecutor would require synchronization.
class BoptBoxFilterNode : public rclcpp::Node
{
public:
  explicit BoptBoxFilterNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void on_fl_scan(sensor_msgs::msg::LaserScan::UniquePtr msg);
  void on_fr_scan(sensor_msgs::msg::LaserScan::UniquePtr msg);
  void on_feedback(std_msgs::msg::Int8MultiArray::ConstSharedPtr msg);

  Box fl_default_;
  Box fl_triggered_;
  Box fr_default_;
  Box fr_triggered_;
  bool triggered_ {false};

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr fl_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr fr_sub_;
  rclcpp::Subscription<std_msgs::msg::Int8MultiArray>::SharedPtr feedback_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr fl_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr fr_pub_;
};

}  // namespace bopt_box_filter

#endif  // BOPT_BOX_FILTER__BOX_FILTER_NODE_HPP_
