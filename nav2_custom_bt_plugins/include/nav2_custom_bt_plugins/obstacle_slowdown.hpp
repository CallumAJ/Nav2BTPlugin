#pragma once

#include <string>
#include "behaviortree_cpp_v3/decorator_node.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav2_msgs/msg/speed_limit.hpp"

namespace nav2_custom_bt_plugins
{

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
