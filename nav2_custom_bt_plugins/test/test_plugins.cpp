#include <gtest/gtest.h>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/msg/speed_limit.hpp"

#include "nav2_custom_bt_plugins/battery_monitor.hpp"
#include "nav2_custom_bt_plugins/crowd_stop.hpp"
#include "nav2_custom_bt_plugins/obstacle_slowdown.hpp"
#include "nav2_custom_bt_plugins/return_to_dock.hpp"

class BTPluginTestFixture : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("test_node");

    factory_ = std::make_unique<BT::BehaviorTreeFactory>();
    factory_->registerNodeType<nav2_custom_bt_plugins::BatteryMonitor>("BatteryMonitor");
    factory_->registerNodeType<nav2_custom_bt_plugins::CrowdStop>("CrowdStop");
    factory_->registerNodeType<nav2_custom_bt_plugins::ObstacleSlowdown>("ObstacleSlowdown");
    factory_->registerNodeType<nav2_custom_bt_plugins::ReturnToDock>("ReturnToDock");

    blackboard_ = BT::Blackboard::create();
    blackboard_->set<rclcpp::Node::SharedPtr>("node", node_);
  }

  void TearDown() override
  {
    node_.reset();
    rclcpp::shutdown();
  }

  sensor_msgs::msg::LaserScan createScan(
    const std::vector<float> & ranges,
    float range_min = 0.1f,
    float range_max = 10.0f)
  {
    sensor_msgs::msg::LaserScan scan;
    scan.ranges = ranges;
    scan.range_min = range_min;
    scan.range_max = range_max;
    scan.angle_min = -M_PI;
    scan.angle_max = M_PI;
    scan.angle_increment = (2.0 * M_PI) / ranges.size();
    return scan;
  }

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<BT::BehaviorTreeFactory> factory_;
  BT::Blackboard::Ptr blackboard_;
};

// =============================================================
// BatteryMonitor Tests
// =============================================================

TEST_F(BTPluginTestFixture, BatteryMonitorReturnsFailureWhenNoMessageReceived)
{
  std::string xml = R"(
    <root main_tree_to_execute="MainTree">
      <BehaviorTree ID="MainTree">
        <BatteryMonitor threshold="0.15"/>
      </BehaviorTree>
    </root>
  )";

  auto tree = factory_->createTreeFromText(xml, blackboard_);
  auto status = tree.tickRoot();
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

TEST_F(BTPluginTestFixture, BatteryMonitorReturnsSuccessWhenBatteryBelowThreshold)
{
  auto battery_pub = node_->create_publisher<sensor_msgs::msg::BatteryState>(
    "/battery_state", 10);

  std::string xml = R"(
    <root main_tree_to_execute="MainTree">
      <BehaviorTree ID="MainTree">
        <BatteryMonitor threshold="0.15"/>
      </BehaviorTree>
    </root>
  )";

  auto tree = factory_->createTreeFromText(xml, blackboard_);

  // Publish low battery
  auto msg = sensor_msgs::msg::BatteryState();
  msg.percentage = 0.10;
  battery_pub->publish(msg);

  // Allow message to propagate
  rclcpp::spin_some(node_);
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  auto status = tree.tickRoot();
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
}

TEST_F(BTPluginTestFixture, BatteryMonitorReturnsFailureWhenBatteryAboveThreshold)
{
  auto battery_pub = node_->create_publisher<sensor_msgs::msg::BatteryState>(
    "/battery_state", 10);

  std::string xml = R"(
    <root main_tree_to_execute="MainTree">
      <BehaviorTree ID="MainTree">
        <BatteryMonitor threshold="0.15"/>
      </BehaviorTree>
    </root>
  )";

  auto tree = factory_->createTreeFromText(xml, blackboard_);

  // Publish healthy battery
  auto msg = sensor_msgs::msg::BatteryState();
  msg.percentage = 0.80;
  battery_pub->publish(msg);

  rclcpp::spin_some(node_);
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  auto status = tree.tickRoot();
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

// =============================================================
// CrowdStop Tests
// =============================================================

TEST_F(BTPluginTestFixture, CrowdStopReturnsSuccessWhenNoCrowd)
{
  auto scan_pub = node_->create_publisher<sensor_msgs::msg::LaserScan>(
    "/scan", 10);

  std::string xml = R"(
    <root main_tree_to_execute="MainTree">
      <BehaviorTree ID="MainTree">
        <CrowdStop density_threshold="0.8" proximity_distance="1.0"/>
      </BehaviorTree>
    </root>
  )";

  auto tree = factory_->createTreeFromText(xml, blackboard_);

  // All readings far away (no crowd)
  auto scan = createScan({5.0f, 5.0f, 5.0f, 5.0f, 5.0f});
  scan_pub->publish(scan);

  rclcpp::spin_some(node_);
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  auto status = tree.tickRoot();
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
}

TEST_F(BTPluginTestFixture, CrowdStopReturnsFailureWhenDensityExceedsThreshold)
{
  auto scan_pub = node_->create_publisher<sensor_msgs::msg::LaserScan>(
    "/scan", 10);

  std::string xml = R"(
    <root main_tree_to_execute="MainTree">
      <BehaviorTree ID="MainTree">
        <CrowdStop density_threshold="0.5" proximity_distance="1.0"/>
      </BehaviorTree>
    </root>
  )";

  auto tree = factory_->createTreeFromText(xml, blackboard_);

  // 4 out of 5 readings close (80% density > 50% threshold)
  auto scan = createScan({0.5f, 0.5f, 0.5f, 0.5f, 5.0f});
  scan_pub->publish(scan);

  rclcpp::spin_some(node_);
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  auto status = tree.tickRoot();
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

TEST_F(BTPluginTestFixture, CrowdStopRespectsProximityDistance)
{
  auto scan_pub = node_->create_publisher<sensor_msgs::msg::LaserScan>(
    "/scan", 10);

  // Large proximity_distance=3.0 means readings at 2.0m count as "close"
  std::string xml = R"(
    <root main_tree_to_execute="MainTree">
      <BehaviorTree ID="MainTree">
        <CrowdStop density_threshold="0.5" proximity_distance="3.0"/>
      </BehaviorTree>
    </root>
  )";

  auto tree = factory_->createTreeFromText(xml, blackboard_);

  // All readings at 2.0m — within proximity_distance=3.0, so 100% density > 50%
  auto scan = createScan({2.0f, 2.0f, 2.0f, 2.0f, 2.0f});
  scan_pub->publish(scan);

  rclcpp::spin_some(node_);
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  auto status = tree.tickRoot();
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

// =============================================================
// ReturnToDock Tests
// =============================================================

TEST_F(BTPluginTestFixture, ReturnToDockParsesXYTheta)
{
  std::string xml = R"(
    <root main_tree_to_execute="MainTree">
      <BehaviorTree ID="MainTree">
        <ReturnToDock dock_pose="1.5;2.5;1.57"/>
      </BehaviorTree>
    </root>
  )";

  auto tree = factory_->createTreeFromText(xml, blackboard_);
  auto status = tree.tickRoot();

  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);

  // Check that the goal was set on the blackboard
  auto goal = blackboard_->get<geometry_msgs::msg::PoseStamped>("goal");
  EXPECT_NEAR(goal.pose.position.x, 1.5, 0.01);
  EXPECT_NEAR(goal.pose.position.y, 2.5, 0.01);
  EXPECT_NEAR(goal.pose.position.z, 0.0, 0.01);
  EXPECT_EQ(goal.header.frame_id, "map");

  // Check quaternion from theta=1.57
  EXPECT_NEAR(goal.pose.orientation.z, std::sin(1.57 / 2.0), 0.01);
  EXPECT_NEAR(goal.pose.orientation.w, std::cos(1.57 / 2.0), 0.01);
}

TEST_F(BTPluginTestFixture, ReturnToDockSetsOriginPose)
{
  std::string xml = R"(
    <root main_tree_to_execute="MainTree">
      <BehaviorTree ID="MainTree">
        <ReturnToDock dock_pose="0.0;0.0;0.0"/>
      </BehaviorTree>
    </root>
  )";

  auto tree = factory_->createTreeFromText(xml, blackboard_);
  auto status = tree.tickRoot();

  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);

  auto goal = blackboard_->get<geometry_msgs::msg::PoseStamped>("goal");
  EXPECT_NEAR(goal.pose.position.x, 0.0, 0.01);
  EXPECT_NEAR(goal.pose.position.y, 0.0, 0.01);
  EXPECT_NEAR(goal.pose.orientation.z, 0.0, 0.01);
  EXPECT_NEAR(goal.pose.orientation.w, 1.0, 0.01);
}

// =============================================================
// ObstacleSlowdown Tests
// =============================================================

TEST_F(BTPluginTestFixture, ObstacleSlowdownPublishesFullSpeedWhenClear)
{
  auto scan_pub = node_->create_publisher<sensor_msgs::msg::LaserScan>(
    "/scan", 10);

  nav2_msgs::msg::SpeedLimit received_speed;
  bool speed_received = false;
  auto speed_sub = node_->create_subscription<nav2_msgs::msg::SpeedLimit>(
    "/speed_limit", 10,
    [&](const nav2_msgs::msg::SpeedLimit::SharedPtr msg) {
      received_speed = *msg;
      speed_received = true;
    });

  // ObstacleSlowdown is a decorator, needs a child node
  std::string xml = R"(
    <root main_tree_to_execute="MainTree">
      <BehaviorTree ID="MainTree">
        <ObstacleSlowdown distance="0.6">
          <AlwaysSuccess/>
        </ObstacleSlowdown>
      </BehaviorTree>
    </root>
  )";

  auto tree = factory_->createTreeFromText(xml, blackboard_);

  // Publish scan with obstacles far away
  auto scan = createScan({5.0f, 5.0f, 5.0f, 5.0f, 5.0f});
  scan_pub->publish(scan);

  rclcpp::spin_some(node_);
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  tree.tickRoot();

  // Process the published speed limit
  rclcpp::spin_some(node_);
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  rclcpp::spin_some(node_);

  ASSERT_TRUE(speed_received);
  EXPECT_TRUE(received_speed.percentage);
  EXPECT_NEAR(received_speed.speed_limit, 100.0, 1.0);
}

TEST_F(BTPluginTestFixture, ObstacleSlowdownReducesSpeedNearObstacle)
{
  auto scan_pub = node_->create_publisher<sensor_msgs::msg::LaserScan>(
    "/scan", 10);

  nav2_msgs::msg::SpeedLimit received_speed;
  bool speed_received = false;
  auto speed_sub = node_->create_subscription<nav2_msgs::msg::SpeedLimit>(
    "/speed_limit", 10,
    [&](const nav2_msgs::msg::SpeedLimit::SharedPtr msg) {
      received_speed = *msg;
      speed_received = true;
    });

  std::string xml = R"(
    <root main_tree_to_execute="MainTree">
      <BehaviorTree ID="MainTree">
        <ObstacleSlowdown distance="0.6">
          <AlwaysSuccess/>
        </ObstacleSlowdown>
      </BehaviorTree>
    </root>
  )";

  auto tree = factory_->createTreeFromText(xml, blackboard_);

  // Publish scan with one close obstacle at 0.3m (50% of 0.6m threshold)
  auto scan = createScan({5.0f, 5.0f, 0.3f, 5.0f, 5.0f});
  scan_pub->publish(scan);

  rclcpp::spin_some(node_);
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  tree.tickRoot();

  rclcpp::spin_some(node_);
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  rclcpp::spin_some(node_);

  ASSERT_TRUE(speed_received);
  EXPECT_TRUE(received_speed.percentage);
  // 0.3 / 0.6 = 50%
  EXPECT_NEAR(received_speed.speed_limit, 50.0, 5.0);
}

TEST_F(BTPluginTestFixture, ObstacleSlowdownClampsMinSpeed)
{
  auto scan_pub = node_->create_publisher<sensor_msgs::msg::LaserScan>(
    "/scan", 10);

  nav2_msgs::msg::SpeedLimit received_speed;
  bool speed_received = false;
  auto speed_sub = node_->create_subscription<nav2_msgs::msg::SpeedLimit>(
    "/speed_limit", 10,
    [&](const nav2_msgs::msg::SpeedLimit::SharedPtr msg) {
      received_speed = *msg;
      speed_received = true;
    });

  std::string xml = R"(
    <root main_tree_to_execute="MainTree">
      <BehaviorTree ID="MainTree">
        <ObstacleSlowdown distance="0.6">
          <AlwaysSuccess/>
        </ObstacleSlowdown>
      </BehaviorTree>
    </root>
  )";

  auto tree = factory_->createTreeFromText(xml, blackboard_);

  // Very close obstacle at 0.05m — speed should clamp to 20% minimum
  auto scan = createScan({5.0f, 0.12f, 5.0f, 5.0f, 5.0f});
  scan_pub->publish(scan);

  rclcpp::spin_some(node_);
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  tree.tickRoot();

  rclcpp::spin_some(node_);
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  rclcpp::spin_some(node_);

  ASSERT_TRUE(speed_received);
  EXPECT_TRUE(received_speed.percentage);
  // 0.12 / 0.6 = 0.2 = 20%, which is the minimum
  EXPECT_NEAR(received_speed.speed_limit, 20.0, 1.0);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
