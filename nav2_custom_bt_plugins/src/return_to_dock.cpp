#include "nav2_custom_bt_plugins/return_to_dock.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include <sstream>

namespace nav2_custom_bt_plugins
{

ReturnToDock::ReturnToDock(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  RCLCPP_INFO(node_->get_logger(), "ReturnToDock: initialized");
}

BT::PortsList ReturnToDock::providedPorts()
{
  return { BT::InputPort<std::string>("dock_pose") };
}

geometry_msgs::msg::PoseStamped ReturnToDock::parseDockPose(
  const std::string & pose_str)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.header.stamp = node_->get_clock()->now();

  // Parse dock position from semicolon-delimited string: "x;y;theta"
  // where x, y are map-frame coordinates and theta is yaw in radians.
  // Example: "0.0;0.0;0.0" = origin facing +X
  std::istringstream ss(pose_str);
  std::string token;
  double x = 0.0, y = 0.0, theta = 0.0;

  if (std::getline(ss, token, ';')) x = std::stod(token);
  if (std::getline(ss, token, ';')) y = std::stod(token);
  if (std::getline(ss, token, ';')) theta = std::stod(token);

  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = 0.0;

  // Convert yaw angle (theta) to a quaternion representing rotation about
  // the Z axis. For a 2D robot only z and w components are needed:
  //   q.z = sin(theta / 2),  q.w = cos(theta / 2)
  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.0;
  pose.pose.orientation.z = std::sin(theta / 2.0);
  pose.pose.orientation.w = std::cos(theta / 2.0);

  return pose;
}

BT::NodeStatus ReturnToDock::tick()
{
  std::string dock_pose_str;
  if (!getInput("dock_pose", dock_pose_str)) {
    RCLCPP_ERROR(node_->get_logger(), "ReturnToDock: missing dock_pose input");
    return BT::NodeStatus::FAILURE;
  }

  auto dock_pose = parseDockPose(dock_pose_str);

  RCLCPP_INFO(node_->get_logger(),
    "ReturnToDock: setting goal to dock at (%.2f, %.2f)",
    dock_pose.pose.position.x, dock_pose.pose.position.y);

  // Write the dock pose to the blackboard as "goal". The downstream
  // ComputePathToPose and FollowPath nodes read {goal} from the blackboard
  // to plan and execute the path to the docking station.
  config().blackboard->set<geometry_msgs::msg::PoseStamped>("goal", dock_pose);

  return BT::NodeStatus::SUCCESS;
}

}
