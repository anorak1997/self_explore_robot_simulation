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

#include "bopt_box_filter/box_filter_node.hpp"

#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "bopt_box_filter/box_filter.hpp"

namespace bopt_box_filter
{

namespace
{
constexpr char kParamFlInput[] = "input_topics.fl";
constexpr char kParamFrInput[] = "input_topics.fr";
constexpr char kParamFlOutput[] = "output_topics.fl";
constexpr char kParamFrOutput[] = "output_topics.fr";
constexpr char kParamFeedbackTopic[] = "feedback_topic";

Box declare_and_load_box(rclcpp::Node * node, const std::string & name)
{
  const auto arr = node->declare_parameter<std::vector<double>>(
    name, std::vector<double>{});
  return box_from_array(name, arr);
}

void require_non_empty(const char * name, const std::string & value)
{
  if (value.empty()) {
    throw std::invalid_argument(
            std::string("required parameter '") + name + "' is empty");
  }
}
}  // namespace

BoptBoxFilterNode::BoptBoxFilterNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("bopt_box_filter", options)
{
  const auto fl_input = declare_parameter<std::string>(kParamFlInput, "");
  const auto fr_input = declare_parameter<std::string>(kParamFrInput, "");
  const auto fl_output = declare_parameter<std::string>(kParamFlOutput, "");
  const auto fr_output = declare_parameter<std::string>(kParamFrOutput, "");
  const auto feedback_topic = declare_parameter<std::string>(
    kParamFeedbackTopic, "");

  require_non_empty(kParamFlInput, fl_input);
  require_non_empty(kParamFrInput, fr_input);
  require_non_empty(kParamFlOutput, fl_output);
  require_non_empty(kParamFrOutput, fr_output);
  require_non_empty(kParamFeedbackTopic, feedback_topic);

  fl_default_ = declare_and_load_box(this, "fl_default");
  fl_triggered_ = declare_and_load_box(this, "fl_triggered");
  fr_default_ = declare_and_load_box(this, "fr_default");
  fr_triggered_ = declare_and_load_box(this, "fr_triggered");

  const auto sensor_qos = rclcpp::SensorDataQoS();
  fl_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(fl_output, sensor_qos);
  fr_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(fr_output, sensor_qos);

  fl_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    fl_input, sensor_qos,
    [this](sensor_msgs::msg::LaserScan::UniquePtr msg) {
      on_fl_scan(std::move(msg));
    });
  fr_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    fr_input, sensor_qos,
    [this](sensor_msgs::msg::LaserScan::UniquePtr msg) {
      on_fr_scan(std::move(msg));
    });
  feedback_sub_ = create_subscription<std_msgs::msg::Int8MultiArray>(
    feedback_topic, rclcpp::QoS(10),
    [this](std_msgs::msg::Int8MultiArray::ConstSharedPtr msg) {
      on_feedback(msg);
    });

  RCLCPP_INFO(
    get_logger(), "bopt_box_filter started; initial state = default");
}

void BoptBoxFilterNode::on_fl_scan(
  sensor_msgs::msg::LaserScan::UniquePtr msg)
{
  const Box & box = triggered_ ? fl_triggered_ : fl_default_;
  apply_box_filter(*msg, box);
  fl_pub_->publish(std::move(msg));
}

void BoptBoxFilterNode::on_fr_scan(
  sensor_msgs::msg::LaserScan::UniquePtr msg)
{
  const Box & box = triggered_ ? fr_triggered_ : fr_default_;
  apply_box_filter(*msg, box);
  fr_pub_->publish(std::move(msg));
}

void BoptBoxFilterNode::on_feedback(
  std_msgs::msg::Int8MultiArray::ConstSharedPtr msg)
{
  if (msg->data.size() < 4) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "feedback msg has %zu elements, need at least 4; ignoring",
      msg->data.size());
    return;
  }
  const int8_t value = msg->data[3];
  if (value != 0 && value != 1) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "feedback data[3]=%d is neither 0 nor 1; keeping state",
      static_cast<int>(value));
    return;
  }
  const bool new_state = update_triggered_state(triggered_, *msg);
  if (new_state != triggered_) {
    RCLCPP_INFO(
      get_logger(), "triggered state changed: %s -> %s",
      triggered_ ? "true" : "false",
      new_state ? "true" : "false");
  }
  triggered_ = new_state;
}

}  // namespace bopt_box_filter
