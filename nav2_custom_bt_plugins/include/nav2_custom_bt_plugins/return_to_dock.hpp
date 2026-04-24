#pragma once

#include <string>
#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace nav2_custom_bt_plugins
{

/**
 * @brief BT action node that sets the navigation goal to the docking station.
 *
 * Parses a dock position from a "x;y;theta" string, converts it to a
 * PoseStamped (with theta converted to a quaternion), and writes it to
 * the blackboard as "goal". Downstream nodes (ComputePathToPose, FollowPath)
 * then navigate the robot to that pose.
 *
 * This is a SyncActionNode — it completes in a single tick with no async work.
 *
 * BT Return Values:
 *   SUCCESS  — goal was successfully set on the blackboard
 *   FAILURE  — dock_pose input was missing
 *
 * Ports:
 *   dock_pose (input, string) — dock position as "x;y;theta" in the map frame
 *                                (e.g. "0.0;0.0;0.0")
 */
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
