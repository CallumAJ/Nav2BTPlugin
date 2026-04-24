#pragma once

#include <string>
#include <vector>
#include "behaviortree_cpp_v3/condition_node.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace nav2_custom_bt_plugins
{

/**
 * @brief BT condition node that halts navigation when a dense crowd is detected.
 *
 * Subscribes to /scan (LaserScan) and estimates crowd density as the ratio
 * of laser readings closer than proximity_distance to the total number of
 * valid readings. If this ratio exceeds the density_threshold, the area is
 * considered too crowded and navigation is halted.
 *
 * BT Return Values:
 *   SUCCESS  — density is below threshold, safe to proceed
 *   FAILURE  — density exceeds threshold, halt navigation
 *
 * Ports:
 *   density_threshold   (input, double) — max allowed density ratio (e.g. 0.7 = 70%)
 *   proximity_distance  (input, double) — distance in meters to count a reading
 *                                          as "close" (e.g. 1.5m)
 */
class CrowdStop : public BT::ConditionNode
{
public:
  CrowdStop(
    const std::string & name,
    const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  double density_threshold_;
  double crowd_density_;
  double proximity_distance_;
};

}
