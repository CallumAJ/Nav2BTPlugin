#include "behaviortree_cpp_v3/bt_factory.h"
#include "nav2_custom_bt_plugins/obstacle_slowdown.hpp"
#include "nav2_custom_bt_plugins/battery_monitor.hpp"
#include "nav2_custom_bt_plugins/crowd_stop.hpp"
#include "nav2_custom_bt_plugins/return_to_dock.hpp"

// Manually defining the symbol that Nav2's loader looks for
extern "C" __attribute__ ((visibility ("default"))) 
void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory& factory)
{
  factory.registerNodeType<nav2_custom_bt_plugins::ObstacleSlowdown>("ObstacleSlowdown");
  factory.registerNodeType<nav2_custom_bt_plugins::BatteryMonitor>("BatteryMonitor");
  factory.registerNodeType<nav2_custom_bt_plugins::CrowdStop>("CrowdStop");
  factory.registerNodeType<nav2_custom_bt_plugins::ReturnToDock>("ReturnToDock");
}