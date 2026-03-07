#pragma once

#include "behaviortree_cpp_v3/condition_node.h"

namespace nav2_custom_bt_plugins
{

class BatteryMonitor : public BT::ConditionNode
{
public:
  BatteryMonitor(
    const std::string & name,
    const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  double threshold_;
};

}