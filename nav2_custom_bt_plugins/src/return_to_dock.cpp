#include "nav2_custom_bt_plugins/return_to_dock.hpp"

namespace nav2_custom_bt_plugins
{

ReturnToDock::ReturnToDock(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList ReturnToDock::providedPorts()
{
  return { BT::InputPort<std::string>("dock_pose") };
}

BT::NodeStatus ReturnToDock::tick()
{
  return BT::NodeStatus::SUCCESS;
}

}