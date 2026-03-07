#pragma once

#include "behaviortree_cpp_v3/condition_node.h"

namespace nav2_custom_bt_plugins
{

class CrowdStop : public BT::ConditionNode
{
public:
  CrowdStop(
    const std::string & name,
    const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  double density_threshold_;
};

}