#pragma once

#include <string>
#include "behaviortree_cpp_v3/decorator_node.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav2_msgs/msg/speed_limit.hpp"

namespace nav2_custom_bt_plugins
{

/**
 * @brief BT decorator node that reduces robot speed based on obstacle proximity.
 *
 * Wraps a child navigation node (e.g. RecoveryNode containing ComputePathToPose
 * + FollowPath). On each tick it reads the latest /scan data, finds the closest
 * obstacle, and publishes a proportional speed limit to /speed_limit before
 * ticking the child.
 *
 * Speed scaling formula:
 *   - If closest obstacle >= slowdown_distance: speed = 100%
 *   - If closest obstacle <  slowdown_distance: speed = (distance / slowdown_distance)
 *   - Minimum speed is clamped to 20% to prevent the robot from freezing
 *
 * BT Return Values:
 *   Forwards the child node's status (SUCCESS / FAILURE / RUNNING)
 *
 * Ports:
 *   distance (input, double) — distance threshold in meters at which slowdown
 *                               begins (e.g. 0.6m)
 */
class ObstacleSlowdown : public BT::DecoratorNode
{
public:
  ObstacleSlowdown(
    const std::string & name,
    const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<nav2_msgs::msg::SpeedLimit>::SharedPtr speed_limit_pub_;
  double slowdown_distance_;
  double min_obstacle_distance_;
  double speed_scale_;
};

}
