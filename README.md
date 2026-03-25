# Nav2BTPlugin 

Battery-Aware, Obstacle-Conscious Navigation in Nav2 Using Behaviour Tree Plugins

## Overview

This project extends the ROS 2 Nav2 navigation stack with custom Behavior Tree plugins that enable battery-aware and safety-aware autonomous navigation. The robot navigates toward user-defined goals while continuously monitoring obstacle proximity and environmental congestion. When battery drops below a configurable threshold, the robot automatically interrupts its mission to return to a predefined docking station.

Built on the default Nav2 NavigateToPose pipeline using BehaviorTree.CPP, costmap-based obstacle avoidance, and Nav2 recovery behaviors. Tested using the TurtleBot3 Burger platform in Gazebo simulation with RViz for visualization and goal setting.

## Custom BT Plugins

| Plugin | Type | Description |
|--------|------|-------------|
| BatteryMonitor | Condition | Monitors battery level and returns SUCCESS when below a configurable threshold |
| CrowdStop | Condition | Detects crowd density and halts navigation when density exceeds a threshold |
| ObstacleSlowdown | Decorator | Reduces robot speed when obstacles are detected within a specified distance |
| ReturnToDock | Action | Navigates the robot back to a predefined docking station |

## Behaviour Tree Structure

The custom behaviour tree uses a Fallback strategy:

1. **Battery check** -- If battery is low, the robot returns to dock
2. **Normal navigation** -- If battery is OK, the robot navigates to its goal with crowd stopping and obstacle slowdown active

## Dependencies

- ROS 2
- Nav2
- BehaviorTree.CPP v3
- TurtleBot3 (simulation)
- Gazebo

## Building

```bash
colcon build --packages-select nav2_custom_bt_plugins
source install/setup.bash
```

## Team

| Name | Role |
|------|------|
| Anthony Trieu | Nav2 integration lead |
| Callum Alexander | Custom BT plugin development |
| Parth Patel | Simulation scenario design |
| Preston Quach | Docking behavior implementation |

## Course

CMPT 419/720 -- Simon Fraser University
