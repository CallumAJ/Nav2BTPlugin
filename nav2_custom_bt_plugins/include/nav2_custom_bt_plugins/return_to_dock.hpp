#pragma once

#include <string>
#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

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

private:
  geometry_msgs::msg::PoseStamped parseDockPose(const std::string & pose_str);

  rclcpp::Node::SharedPtr node_;
};

}
