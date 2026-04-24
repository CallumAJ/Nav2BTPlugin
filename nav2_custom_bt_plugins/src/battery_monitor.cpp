#include "nav2_custom_bt_plugins/battery_monitor.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace nav2_custom_bt_plugins
{

BatteryMonitor::BatteryMonitor(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config),
  battery_level_(1.0),
  battery_received_(false)
{
  getInput("threshold", threshold_);

  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  battery_sub_ = node_->create_subscription<sensor_msgs::msg::BatteryState>(
    "/battery_state", 10,
    std::bind(&BatteryMonitor::batteryCallback, this, std::placeholders::_1));

  RCLCPP_INFO(node_->get_logger(), "BatteryMonitor: threshold=%.2f", threshold_);
}

BT::PortsList BatteryMonitor::providedPorts()
{
  return { BT::InputPort<double>("threshold") };
}

void BatteryMonitor::batteryCallback(
  const sensor_msgs::msg::BatteryState::SharedPtr msg)
{
  battery_level_ = msg->percentage;
  battery_received_ = true;
}

BT::NodeStatus BatteryMonitor::tick()
{
  // Process pending subscription callbacks
  rclcpp::spin_some(node_);

  if (!battery_received_) {
    // No battery data received yet — assume battery is OK and let the
    // ReactiveFallback fall through to normal navigation (Priority 2).
    return BT::NodeStatus::FAILURE;
  }

  if (battery_level_ < threshold_) {
    RCLCPP_WARN(node_->get_logger(),
      "BatteryMonitor: LOW BATTERY %.1f%% < threshold %.1f%%",
      battery_level_ * 100.0, threshold_ * 100.0);
    // SUCCESS here means "condition is true" (battery IS low), which causes
    // the parent Sequence to continue into the docking branch.
    return BT::NodeStatus::SUCCESS;
  }

  // FAILURE means "condition is false" (battery is healthy). In the
  // ReactiveFallback this causes the tree to skip the docking branch
  // and try the next child (normal navigation).
  return BT::NodeStatus::FAILURE;
}

}
