#pragma once

#include "behaviortree_cpp_v3/decorator_node.h"

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
  double slowdown_distance_;
};

}