#include "nav2_custom_bt_plugins/crowd_stop.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include <cmath>
#include <algorithm>

namespace nav2_custom_bt_plugins
{

CrowdStop::CrowdStop(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config),
  crowd_density_(0.0),
  proximity_distance_(1.0)
{
  getInput("density_threshold", density_threshold_);
  getInput("proximity_distance", proximity_distance_);

  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  scan_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", 10,
    std::bind(&CrowdStop::scanCallback, this, std::placeholders::_1));

  RCLCPP_INFO(node_->get_logger(),
    "CrowdStop: density_threshold=%.2f, proximity_distance=%.2f",
    density_threshold_, proximity_distance_);
}

BT::PortsList CrowdStop::providedPorts()
{
  return {
    BT::InputPort<double>("density_threshold"),
    BT::InputPort<double>("proximity_distance")
  };
}

void CrowdStop::scanCallback(
  const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  int close_count = 0;
  int valid_count = 0;

  for (size_t i = 0; i < msg->ranges.size(); ++i) {
    float range = msg->ranges[i];
    if (std::isfinite(range) && range >= msg->range_min && range <= msg->range_max) {
      valid_count++;
      if (range < proximity_distance_) {
        close_count++;
      }
    }
  }

  if (valid_count > 0) {
    crowd_density_ = static_cast<double>(close_count) / static_cast<double>(valid_count);
  } else {
    crowd_density_ = 0.0;
  }
}

BT::NodeStatus CrowdStop::tick()
{
  // Process pending subscription callbacks
  rclcpp::spin_some(node_);

  // SUCCESS = safe to proceed (density below threshold)
  // FAILURE = too crowded, halt navigation
  if (crowd_density_ > density_threshold_) {
    RCLCPP_WARN(node_->get_logger(),
      "CrowdStop: HIGH DENSITY %.2f > threshold %.2f - halting navigation",
      crowd_density_, density_threshold_);
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::SUCCESS;
}

}
