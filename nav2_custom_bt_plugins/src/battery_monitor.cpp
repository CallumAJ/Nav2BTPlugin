#include "nav2_custom_bt_plugins/battery_monitor.hpp"

namespace nav2_custom_bt_plugins
{

BatteryMonitor::BatteryMonitor(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
  getInput("threshold", threshold_);
}

BT::PortsList BatteryMonitor::providedPorts()
{
  return { BT::InputPort<double>("threshold") };
}

BT::NodeStatus BatteryMonitor::tick()
{
  double fake_battery = 0.5;

  if(fake_battery < threshold_)
    return BT::NodeStatus::SUCCESS;

  return BT::NodeStatus::FAILURE;
}

}