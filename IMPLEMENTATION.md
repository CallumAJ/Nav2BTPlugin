# Plugin Implementation Guide

This document explains the implementation of each custom Behavior Tree plugin, how they integrate with Nav2, and how the behavior tree orchestrates them.

## Architecture Overview

The system consists of four custom BT plugins compiled into a single shared library (`libnav2_custom_bt_plugins.so`). Nav2's `bt_navigator` loads this library at startup and makes the nodes available for use in behavior tree XML files.

All plugins obtain a shared `rclcpp::Node` from the BT blackboard (set by Nav2's `BtActionServer` as `"node"`). Since this node is not spun by any executor during BT execution, each plugin calls `rclcpp::spin_some(node_)` in its `tick()` method to process pending subscription callbacks.

```
bt_navigator
  └── BehaviorTreeEngine
        └── custom_nav_tree.xml
              ├── BatteryMonitor    (subscribes /battery_state)
              ├── ReturnToDock      (writes {goal} to blackboard)
              ├── CrowdStop         (subscribes /scan)
              └── ObstacleSlowdown  (subscribes /scan, publishes /speed_limit)
```

---

## BatteryMonitor

**Type:** `BT::ConditionNode`
**Files:** `battery_monitor.hpp`, `battery_monitor.cpp`

### Purpose

Monitors the robot's battery level and signals when it drops below a configurable threshold. Used as the trigger for initiating a return-to-dock sequence.

### ROS Interface

| Direction | Topic | Message Type |
|-----------|-------|-------------|
| Subscribes | `/battery_state` | `sensor_msgs/msg/BatteryState` |

### BT Ports

| Port | Type | Direction | Description |
|------|------|-----------|-------------|
| `threshold` | double | Input | Battery percentage (0.0--1.0) below which to trigger |

### How It Works

```
Constructor:
  - Reads threshold from BT input port
  - Gets shared node from blackboard
  - Creates subscription to /battery_state

batteryCallback(msg):
  - Stores msg.percentage in battery_level_
  - Sets battery_received_ = true

tick():
  - spin_some(node_)          // process pending callbacks
  - if !battery_received_     -> return FAILURE (no data yet, assume OK)
  - if battery_level_ < threshold_ -> log warning, return SUCCESS (low battery)
  - else                      -> return FAILURE (battery is fine)
```

### Return Value Semantics

- **SUCCESS** = battery is low (triggers downstream dock sequence)
- **FAILURE** = battery is fine (BT skips the dock branch)

This is intentional: in the BT tree, BatteryMonitor is the first child of a Sequence. Returning SUCCESS means "condition met, continue the sequence" (proceed to ReturnToDock). Returning FAILURE means "condition not met, skip this branch."

---

## CrowdStop

**Type:** `BT::ConditionNode`
**Files:** `crowd_stop.hpp`, `crowd_stop.cpp`

### Purpose

Detects when the robot is surrounded by too many close obstacles (a "crowd") and halts navigation to prevent unsafe movement in congested areas.

### ROS Interface

| Direction | Topic | Message Type |
|-----------|-------|-------------|
| Subscribes | `/scan` | `sensor_msgs/msg/LaserScan` |

### BT Ports

| Port | Type | Direction | Description |
|------|------|-----------|-------------|
| `density_threshold` | double | Input | Fraction of scan points (0.0--1.0) that must be within proximity distance |

### How It Works

```
Constructor:
  - Reads density_threshold from BT input port
  - Sets proximity_distance_ = 1.0m (hardcoded)
  - Creates subscription to /scan

scanCallback(msg):
  - For each valid laser range:
    - Count total valid_count
    - Count ranges < proximity_distance_ as close_count
  - crowd_density_ = close_count / valid_count

tick():
  - spin_some(node_)
  - if crowd_density_ > density_threshold_ -> log warning, return FAILURE (halt)
  - else                                   -> return SUCCESS (safe to proceed)
```

### Return Value Semantics

- **SUCCESS** = safe to proceed (density below threshold)
- **FAILURE** = too crowded (halts the navigation Sequence)

Opposite convention from BatteryMonitor: here SUCCESS means "condition OK, continue navigating."

### Density Calculation

The density is the ratio of laser scan points within 1.0m to total valid scan points. With the default threshold of 0.8, the robot must have 80% of its scan readings blocked by nearby obstacles before halting. This only triggers in very tight or enclosed spaces.

---

## ObstacleSlowdown

**Type:** `BT::DecoratorNode`
**Files:** `obstacle_slowdown.hpp`, `obstacle_slowdown.cpp`

### Purpose

Dynamically reduces the robot's speed as it approaches obstacles. Wraps the navigation pipeline (ComputePathToPose + FollowPath) and publishes speed limit messages that Nav2's controller server uses to scale velocity.

### ROS Interface

| Direction | Topic | Message Type |
|-----------|-------|-------------|
| Subscribes | `/scan` | `sensor_msgs/msg/LaserScan` |
| Publishes | `/speed_limit` | `nav2_msgs/msg/SpeedLimit` |

### BT Ports

| Port | Type | Direction | Description |
|------|------|-----------|-------------|
| `distance` | double | Input | Distance in meters at which slowdown begins |

### How It Works

```
Constructor:
  - Reads slowdown distance from BT input port
  - Creates subscription to /scan
  - Creates publisher for /speed_limit

scanCallback(msg):
  - Finds minimum range across all valid laser readings
  - Computes speed_scale:
    - If min_distance < slowdown_distance:
        speed_scale = max(0.2, min_distance / slowdown_distance)
    - Else:
        speed_scale = 1.0

tick():
  - spin_some(node_)
  - Publishes SpeedLimit message:
      percentage = true
      speed_limit = speed_scale * 100.0
  - Executes child node (the navigation pipeline)
  - Returns child's status (SUCCESS / FAILURE / RUNNING)
```

### Speed Scaling

The speed scales linearly from 100% at the slowdown distance to 20% minimum:

```
Distance from obstacle    Speed scale
─────────────────────    ───────────
>= 0.6m (threshold)      100%
   0.3m                   50%
   0.12m                  20% (minimum)
< 0.12m                   20% (clamped)
```

### Why a Decorator

As a `DecoratorNode`, ObstacleSlowdown wraps the RecoveryNode that contains ComputePathToPose + FollowPath. On every BT tick, it:
1. Publishes the current speed limit
2. Ticks the child navigation pipeline
3. Passes through the child's return status

This means speed limits are updated continuously while the robot navigates.

---

## ReturnToDock

**Type:** `BT::SyncActionNode`
**Files:** `return_to_dock.hpp`, `return_to_dock.cpp`

### Purpose

When triggered by a low battery condition, sets the navigation goal to a predefined docking station. Instead of sending its own navigation action (which would conflict with the running bt_navigator), it writes the dock pose directly to the BT blackboard so the standard ComputePathToPose + FollowPath pipeline handles the actual navigation.

### ROS Interface

No ROS topics -- communicates entirely through the BT blackboard.

### BT Ports

| Port | Type | Direction | Description |
|------|------|-----------|-------------|
| `dock_pose` | string | Input | Dock position as `"x;y;theta"` (semicolon-delimited) |

### How It Works

```
tick():
  - Gets dock_pose string from BT input port
  - Calls parseDockPose(dock_pose_str):
      - Splits string by ';' into x, y, theta
      - Creates PoseStamped with frame_id = "map"
      - Converts theta to quaternion: z = sin(theta/2), w = cos(theta/2)
  - Sets blackboard entry "goal" to the parsed PoseStamped
  - Returns SUCCESS

parseDockPose("0.0;0.0;0.0"):
  -> PoseStamped { position: (0, 0, 0), orientation: (0, 0, 0, 1), frame: "map" }
```

### Why SyncActionNode

ReturnToDock is synchronous because it only needs to set a blackboard value -- there is no long-running operation. It completes in a single tick:

1. BatteryMonitor returns SUCCESS (low battery)
2. ReturnToDock writes dock pose to `{goal}`, returns SUCCESS
3. ComputePathToPose reads `{goal}` from blackboard, plans path to dock
4. FollowPath executes the path

This avoids a recursive call problem: if ReturnToDock tried to send a `NavigateToPose` action goal, it would be sending it to the same `bt_navigator` that is already running this BT, causing a conflict.

---

## Plugin Registration

**File:** `register_nodes.cpp`

All four plugins are registered in a single file using the `BT_REGISTER_NODES` macro:

```cpp
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<BatteryMonitor>("BatteryMonitor");
  factory.registerNodeType<CrowdStop>("CrowdStop");
  factory.registerNodeType<ObstacleSlowdown>("ObstacleSlowdown");
  factory.registerNodeType<ReturnToDock>("ReturnToDock");
}
```

This is centralized in one file because the macro expands to a function named `BT_RegisterNodesFromPlugin`. Having it in multiple `.cpp` files within the same shared library would cause duplicate symbol errors.

The CMakeLists.txt includes `target_compile_definitions(nav2_custom_bt_plugins PRIVATE BT_PLUGIN_EXPORT)` so the registration function is exported with external linkage (without this, the macro produces a `static` function that Nav2 cannot find when loading the plugin).

---

## Behavior Tree Structure

```xml
<ReactiveFallback>
  <!-- Branch 1: Low battery -> dock -->
  <Sequence>
    <BatteryMonitor threshold="0.15"/>
    <ReturnToDock dock_pose="0.0;0.0;0.0"/>
    <RecoveryNode>
      <ComputePathToPose/> + <FollowPath/>   (navigate to dock)
      <ClearCostmaps/>                        (recovery)
    </RecoveryNode>
  </Sequence>

  <!-- Branch 2: Normal navigation -->
  <Sequence>
    <CrowdStop density_threshold="0.8"/>
    <ObstacleSlowdown distance="0.6">
      <RecoveryNode>
        <ComputePathToPose/> + <FollowPath/> (navigate to goal)
        <ClearCostmaps/>                      (recovery)
      </RecoveryNode>
    </ObstacleSlowdown>
  </Sequence>
</ReactiveFallback>
```

### Why ReactiveFallback

A standard `Fallback` only re-evaluates earlier children after they return SUCCESS or FAILURE. Once Branch 2 starts returning RUNNING (robot is navigating), it would never re-check Branch 1 (battery).

`ReactiveFallback` re-evaluates from the first child on every tick. This means:
- While navigating normally, BatteryMonitor is checked every tick (~10ms)
- The moment battery drops below threshold, Branch 1 takes over mid-navigation
- Branch 2's running navigation is preempted and the robot heads to dock

### Tick-by-Tick Execution (Normal Battery)

```
Tick 1:  BatteryMonitor -> FAILURE (battery OK)
         Fallback moves to Branch 2
         CrowdStop -> SUCCESS (not crowded)
         ObstacleSlowdown publishes speed limit, ticks child
         ComputePathToPose -> SUCCESS (path computed)
         FollowPath -> RUNNING (navigating)
         Branch 2 -> RUNNING

Tick 2+: BatteryMonitor -> FAILURE (still OK)
         Branch 2 resumes -> FollowPath still RUNNING
         ...until FollowPath -> SUCCESS (goal reached)
```

### Tick-by-Tick Execution (Low Battery Mid-Navigation)

```
Tick N:   BatteryMonitor -> SUCCESS (low battery!)
          ReturnToDock -> SUCCESS (sets {goal} to dock)
          ComputePathToPose -> SUCCESS (path to dock)
          FollowPath -> RUNNING (heading to dock)
          Branch 1 -> RUNNING

Tick N+1: BatteryMonitor -> SUCCESS (still low)
          Branch 1 resumes -> FollowPath still RUNNING
          ...until robot reaches dock
```
