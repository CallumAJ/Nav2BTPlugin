#include "nav2_custom_bt_plugins/obstacle_slowdown.hpp"
#include "nav2_msgs/msg/speed_limit.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include <cmath>
#include <algorithm>

namespace nav2_custom_bt_plugins
{

ObstacleSlowdown::ObstacleSlowdown(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::DecoratorNode(name, config),
  min_obstacle_distance_(std::numeric_limits<double>::max()),
  speed_scale_(1.0)
{
  getInput("distance", slowdown_distance_);

  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  scan_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", 10,
    std::bind(&ObstacleSlowdown::scanCallback, this, std::placeholders::_1));

  speed_limit_pub_ = node_->create_publisher<nav2_msgs::msg::SpeedLimit>(
    "/speed_limit", 10);

  RCLCPP_INFO(node_->get_logger(),
    "ObstacleSlowdown: slowdown_distance=%.2f", slowdown_distance_);
}

BT::PortsList ObstacleSlowdown::providedPorts()
{
  return { BT::InputPort<double>("distance") };
}

void ObstacleSlowdown::scanCallback(
  const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  double min_range = std::numeric_limits<double>::max();

  for (size_t i = 0; i < msg->ranges.size(); ++i) {
    float range = msg->ranges[i];
    if (std::isfinite(range) && range >= msg->range_min && range <= msg->range_max) {
      if (range < min_range) {
        min_range = range;
      }
    }
  }

  min_obstacle_distance_ = min_range;

  // Compute speed scaling: 1.0 at slowdown_distance_, 0.2 at very close range
  if (min_obstacle_distance_ < slowdown_distance_) {
    speed_scale_ = std::max(0.2,
      min_obstacle_distance_ / slowdown_distance_);
  } else {
    speed_scale_ = 1.0;
  }
}

BT::NodeStatus ObstacleSlowdown::tick()
{
  // Process pending subscription callbacks
  rclcpp::spin_some(node_);

  setStatus(BT::NodeStatus::RUNNING);

  if (speed_scale_ < 1.0) {
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
      "ObstacleSlowdown: obstacle at %.2fm, speed scale: %.1f%%",
      min_obstacle_distance_, speed_scale_ * 100.0);
  }

  // Publish speed limit on /speed_limit topic for the controller to use
  auto speed_limit_msg = nav2_msgs::msg::SpeedLimit();
  speed_limit_msg.percentage = true;
  speed_limit_msg.speed_limit = speed_scale_ * 100.0;
  speed_limit_pub_->publish(speed_limit_msg);

  const BT::NodeStatus child_status = child_node_->executeTick();

  switch (child_status)
  {
    case BT::NodeStatus::SUCCESS:
      return BT::NodeStatus::SUCCESS;
    case BT::NodeStatus::FAILURE:
      return BT::NodeStatus::FAILURE;
    case BT::NodeStatus::RUNNING:
      return BT::NodeStatus::RUNNING;
    default:
      return BT::NodeStatus::FAILURE;
  }
}

}
