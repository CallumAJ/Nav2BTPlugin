#include "nav2_custom_bt_plugins/crowd_stop.hpp"

namespace nav2_custom_bt_plugins
{

CrowdStop::CrowdStop(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
  getInput("density_threshold", density_threshold_);
}

BT::PortsList CrowdStop::providedPorts()
{
  return { BT::InputPort<double>("density_threshold") };
}

BT::NodeStatus CrowdStop::tick()
{
  double fake_density = 0.2;

  if(fake_density > density_threshold_)
    return BT::NodeStatus::SUCCESS;

  return BT::NodeStatus::FAILURE;
}

}