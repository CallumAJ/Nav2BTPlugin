# Nav2BTPlugin

Battery-Aware, Obstacle-Conscious Navigation in Nav2 Using Behaviour Tree Plugins

## Overview

This project extends the ROS 2 Nav2 navigation stack with custom Behavior Tree plugins that enable battery-aware and safety-aware autonomous navigation. The robot navigates toward user-defined goals while continuously monitoring obstacle proximity and environmental congestion. When battery drops below a configurable threshold, the robot automatically interrupts its mission to return to a predefined docking station.

Built on the default Nav2 NavigateToPose pipeline using BehaviorTree.CPP, costmap-based obstacle avoidance, and Nav2 recovery behaviors. Tested using the TurtleBot3 Waffle platform in Gazebo simulation.

## Intended Behavior

The robot receives a navigation goal and travels to it while four behavior layers run concurrently in priority order:

1. **Battery check** -- If battery drops below 15%, the robot abandons its current mission and returns to the docking station at (0, 0), with all safety behaviors still active during the return
2. **Crowd halt** -- If >70% of lidar readings within 1.5m are blocked (dense crowd or bottleneck), the robot halts and waits for the area to clear
3. **Obstacle slowdown** -- Within 0.6m of an obstacle, speed scales linearly down to 20% minimum -- closer means slower
4. **Normal navigation** -- Otherwise, the robot follows its planned path using Nav2's default planner and controller with recovery behaviors

These priorities are enforced by a `ReactiveFallback` node that re-evaluates from the top on every tick, so higher-priority conditions interrupt immediately.

## Custom BT Plugins

| Plugin | Type | Description |
|--------|------|-------------|
| BatteryMonitor | Condition | Monitors battery level and returns SUCCESS when below a configurable threshold |
| CrowdStop | Condition | Detects crowd density and halts navigation when density exceeds a threshold |
| ObstacleSlowdown | Decorator | Reduces robot speed when obstacles are detected within a specified distance |
| ReturnToDock | Action | Sets the navigation goal to a predefined docking station pose |

## Behaviour Tree Structure

The custom behaviour tree uses a ReactiveFallback strategy (re-evaluates on every tick):

1. **Battery check** -- If battery is low, the robot returns to dock via ComputePathToPose + FollowPath
2. **Normal navigation** -- If battery is OK, the robot navigates to its goal with crowd stopping and obstacle slowdown active

## Dependencies

- ROS 2 Humble
- Nav2 (`nav2_bringup`, `nav2_behavior_tree`, `nav2_bt_navigator`)
- BehaviorTree.CPP v3
- TurtleBot3 packages (`turtlebot3_gazebo`)
- Gazebo

## Prerequisites

Install TurtleBot3 and Nav2 packages (if not already installed):

```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-turtlebot3-gazebo ros-humble-turtlebot3-description
```

## Workspace Setup

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src

# Clone the BT plugin package
git clone https://github.com/CallumAJ/Nav2BTPlugin.git

# Clone the launch/integration package
git clone https://github.com/CallumAJ/nav2_bt_project.git
```

The workspace should look like:

```
ros2_ws/
└── src/
    ├── Nav2BTPlugin/
    │   └── nav2_custom_bt_plugins/   # C++ plugin library
    └── nav2_bt_project/              # Launch files, params, battery sim
```

## Building

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select nav2_custom_bt_plugins nav2_bt_project
source install/setup.bash
```

## Running

### 1. Set environment variables

```bash
export TURTLEBOT3_MODEL=waffle
export ROS_LOCALHOST_ONLY=1
export FASTRTPS_DEFAULT_PROFILES_FILE=$(ros2 pkg prefix nav2_bt_project)/share/nav2_bt_project/params/fastdds_profile.xml
```

The FastDDS profile disables shared memory transport, which prevents DDS discovery issues in containers and Docker environments.

### 2. Launch the full stack

```bash
ros2 launch nav2_bt_project nav2_bt_bringup.launch.py
```

This brings up (in order with timed delays):
- Gazebo with TurtleBot3 Waffle
- Nav2 localization (map_server + AMCL) after 8 seconds
- Nav2 navigation stack with custom BT after 15 seconds
- Battery simulator node

### 3. Verify all nodes are active

```bash
ros2 lifecycle get /map_server      # expect: active [3]
ros2 lifecycle get /amcl            # expect: active [3]
ros2 lifecycle get /bt_navigator    # expect: active [3]
```

### 4. Send a navigation goal

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

## Testing the Plugins

### ObstacleSlowdown

While the robot navigates, monitor the speed limit topic:

```bash
ros2 topic echo /speed_limit
```

When the robot approaches obstacles within 0.6m, you should see speed limit values drop below 100%. The BT logs will show messages like:

```
ObstacleSlowdown: obstacle at 0.30m, speed scale: 50.3%
```

### BatteryMonitor + ReturnToDock

Kill the default battery simulator and start one with low initial battery:

```bash
# In a new terminal (with env vars set)
ros2 run nav2_bt_project battery_simulator --ros-args \
  -p initial_battery_level:=0.10 \
  -p drain_rate:=0.001 \
  -p use_sim_time:=true
```

Then send a navigation goal. Instead of navigating to the requested pose, the robot will return to the dock at (0, 0):

```
BatteryMonitor: LOW BATTERY 9.4% < threshold 15.0%
ReturnToDock: setting goal to dock at (0.00, 0.00)
```

### CrowdStop

CrowdStop triggers when more than 80% of laser scan points are within 1.0m. This occurs in very tight or enclosed spaces. When triggered, the logs will show:

```
CrowdStop: HIGH DENSITY 0.85 > threshold 0.80 - halting navigation
```

## Monitoring

Useful commands for observing system behavior:

```bash
# Battery level
ros2 topic echo /battery_state --field percentage

# Robot position
ros2 topic echo /amcl_pose --field pose.pose.position

# All custom plugin logs
ros2 topic echo /rosout --field msg | grep -iE "battery|dock|crowd|obstacle|speed"

# Velocity commands
ros2 topic echo /cmd_vel --field linear
```

## Configuration

Key parameters can be adjusted in the behavior tree XML (`behavior_trees/custom_nav_tree.xml`):

| Parameter | Default | Description |
|-----------|---------|-------------|
| `BatteryMonitor threshold` | 0.15 | Battery percentage below which dock return triggers |
| `CrowdStop density_threshold` | 0.8 | Fraction of scan points within proximity to trigger halt |
| `ObstacleSlowdown distance` | 0.6 | Distance (meters) at which speed reduction begins |
| `ReturnToDock dock_pose` | 0.0;0.0;0.0 | Dock position as x;y;theta |

Battery simulator parameters (set via `--ros-args -p`):

| Parameter | Default | Description |
|-----------|---------|-------------|
| `initial_battery_level` | 1.0 | Starting battery percentage (0.0 to 1.0) |
| `drain_rate` | 0.005 | Battery drain per second |
| `low_battery_threshold` | 0.15 | Threshold for low battery warnings |

## Team

| Name | Role |
|------|------|
| Anthony Trieu | Nav2 integration lead |
| Callum Alexander | Custom BT plugin development |
| Parth Patel | Simulation scenario design |
| Preston Quach | Docking behavior implementation |

## Course

CMPT 419/720 -- Simon Fraser University
