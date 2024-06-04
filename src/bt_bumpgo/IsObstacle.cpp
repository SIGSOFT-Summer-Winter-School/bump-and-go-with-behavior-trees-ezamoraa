// Copyright 2024 Intelligent Robotics Lab
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

#include <string>
#include <utility>

#include "bt_bumpgo/IsObstacle.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "rclcpp/rclcpp.hpp"

namespace bt_bumpgo
{

using namespace std::chrono_literals;
using namespace std::placeholders;

IsObstacle::IsObstacle(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(xml_tag_name, conf)
{
  config().blackboard->get("node", node_);

  // Complete here: Initialize laser_sub_ subscribing to /input_scan
  laser_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
    "input_scan", rclcpp::SensorDataQoS(), std::bind(&IsObstacle::laser_callback, this, _1));
}

void
IsObstacle::laser_callback(sensor_msgs::msg::LaserScan::UniquePtr msg)
{
  // Complete here: Store last_scan_
  last_scan_ = std::move(msg);
}

BT::NodeStatus
IsObstacle::tick()
{
  if(last_scan_==nullptr)
  {
    return BT::NodeStatus::FAILURE;
  }

  double distance_thresh = 1.0;
  getInput("distance", distance_thresh);

  // Get the index of nearest obstacle
  auto start_range_idx = (last_scan_->ranges.size() / 2) - 0.05 * last_scan_->ranges.size();
  auto end_range_idx = (last_scan_->ranges.size() / 2) + 0.05 * last_scan_->ranges.size();
  int min_idx = std::min_element(
    last_scan_->ranges.begin() + start_range_idx,
    last_scan_->ranges.begin() + end_range_idx) - last_scan_->ranges.begin();

  // Get the distance to nearest obstacle
  float distance_min = last_scan_->ranges[min_idx];

  if (distance_min < distance_thresh) {
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}

}  // namespace bt_bumpgo

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<bt_bumpgo::IsObstacle>("IsObstacle");
}
