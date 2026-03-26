#include "nav2_custom_bt_plugins/obstacle_slowdown.hpp"
#include <rclcpp/rclcpp.hpp>

namespace nav2_custom_bt_plugins
{

ObstacleSlowdown::ObstacleSlowdown(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::DecoratorNode(name, config)
{
  RCLCPP_ERROR(rclcpp::get_logger("ObstacleSlowdown"), "LOADED OBSTACLE PLUGIN");
  getInput("slowdown_distance", slowdown_distance_);
}

BT::PortsList ObstacleSlowdown::providedPorts()
{
  return { BT::InputPort<double>("slowdown_distance") };
}

BT::NodeStatus ObstacleSlowdown::tick()
{
  RCLCPP_ERROR(rclcpp::get_logger("ObstacleSlowdown"), "PLUGIN IS RUNNING");
  if (!child_node_)
    return BT::NodeStatus::FAILURE;

  // Fake obstacle distance for now (Week 2 prototype)
  double obstacle_distance = 0.4;

  if (obstacle_distance < slowdown_distance_)
  {
    RCLCPP_WARN(rclcpp::get_logger("ObstacleSlowdown"),
      "Obstacle close! Slowing down...");
  }

  return child_node_->executeTick();
}

}