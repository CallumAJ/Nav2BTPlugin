#pragma once

#include "behaviortree_cpp_v3/action_node.h"

namespace nav2_custom_bt_plugins
{

class ReturnToDock : public BT::SyncActionNode
{
public:
  ReturnToDock(
    const std::string & name,
    const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

}