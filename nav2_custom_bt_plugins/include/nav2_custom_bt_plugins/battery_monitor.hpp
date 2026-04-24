#pragma once

#include <string>
#include "behaviortree_cpp_v3/condition_node.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"

namespace nav2_custom_bt_plugins
{

/**
 * @brief BT condition node that monitors the robot's battery level.
 *
 * Subscribes to the /battery_state topic and compares the current battery
 * percentage against a configurable threshold. Used as the first check in
 * a ReactiveFallback: when the battery is low, this node returns SUCCESS
 * to activate the docking sequence; otherwise it returns FAILURE so the
 * fallback continues to normal navigation.
 *
 * BT Return Values:
 *   SUCCESS  — battery is below threshold (triggers docking branch)
 *   FAILURE  — battery is healthy or no data received yet
 *
 * Ports:
 *   threshold (input, double) — battery level below which docking is triggered
 *                                (e.g. 0.15 = 15%)
 */
class BatteryMonitor : public BT::ConditionNode
{
public:
  BatteryMonitor(
    const std::string & name,
    const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  void batteryCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
  double threshold_;
  double battery_level_;
  bool battery_received_;
};

}
