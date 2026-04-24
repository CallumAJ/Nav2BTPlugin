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
  // Find the closest valid obstacle reading from the lidar scan
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

  // Compute speed scaling as a linear proportion of obstacle distance:
  //   speed_scale = obstacle_dist / slowdown_distance
  //
  // Examples (with slowdown_distance = 0.6m):
  //   obstacle at 0.6m+ → scale = 1.0  (full speed)
  //   obstacle at 0.3m  → scale = 0.5  (half speed)
  //   obstacle at 0.1m  → scale = 0.2  (clamped to minimum 20%)
  //
  // The 0.2 floor prevents the robot from stopping entirely, which would
  // make it unresponsive; full stops are handled by CrowdStop instead.
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

  // Log speed reduction at most once every 2 seconds to avoid flooding
  if (speed_scale_ < 1.0) {
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
      "ObstacleSlowdown: obstacle at %.2fm, speed scale: %.1f%%",
      min_obstacle_distance_, speed_scale_ * 100.0);
  }

  // Publish the computed speed limit so the Nav2 controller server can
  // scale its velocity commands accordingly
  auto speed_limit_msg = nav2_msgs::msg::SpeedLimit();
  speed_limit_msg.percentage = true;
  speed_limit_msg.speed_limit = speed_scale_ * 100.0;
  speed_limit_pub_->publish(speed_limit_msg);

  // As a decorator, tick the child node (navigation pipeline) and forward
  // its result — the speed limit is applied independently via the topic
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
