# TODO

## Known Issues

### Battery Simulator Crashes on Relaunch
The battery simulator launched via `nav2_bt_bringup.launch.py` crashes with `RCLError: the given context is not valid` when Gazebo restarts into an existing session. The node works correctly when started manually in a separate terminal. The launch-managed instance inherits a stale ROS context from a previous run.

**Workaround:** Start the battery simulator manually after launch:
```bash
ros2 run nav2_bt_project battery_simulator --ros-args \
  -p initial_battery_level:=1.0 -p drain_rate:=0.005 -p use_sim_time:=true
```

### Speed Limit May Not Affect DWB Controller
ObstacleSlowdown publishes to `/speed_limit` (`nav2_msgs/msg/SpeedLimit`), but Nav2's default DWB local planner does not natively subscribe to this topic. The messages are published correctly, but actual velocity reduction depends on controller configuration. Options to fix:

- Switch to `RegulatedPurePursuitController`, which reads `/speed_limit` out of the box
- Add a `SpeedController` BT decorator from Nav2's built-in plugins
- Write a velocity scaling node that subscribes to `/speed_limit` and scales `/cmd_vel`

### CrowdStop Is Hard to Trigger
The default density threshold of 0.8 requires 80% of laser scan points to be within 1.0m. This rarely happens in the turtlebot3_world map. The plugin logic is correct but the threshold is too high for practical testing in open environments.

**Possible fixes:**
- Lower `density_threshold` in the behavior tree XML (e.g., 0.4)
- Make `proximity_distance` a configurable BT input port instead of hardcoded at 1.0m
- Test in a custom world with tighter corridors

## Improvements

### Add Unit Tests
The plugins have no automated tests. Adding gtest-based tests with mock ROS publishers would catch regressions and make the project more robust.

Tests to write:
- BatteryMonitor returns SUCCESS when battery message is below threshold
- BatteryMonitor returns FAILURE when no message received yet
- CrowdStop returns FAILURE when density exceeds threshold
- ObstacleSlowdown publishes correct speed scale values
- ReturnToDock correctly parses "x;y;theta" and sets blackboard goal

### Push nav2_bt_project Changes
The `nav2_bt_project` package (launch files, params, battery simulator, behavior tree XML) has been modified but not pushed to a remote repository. Key changes that need to be tracked:

- `behavior_trees/custom_nav_tree.xml` — changed to ReactiveFallback
- `params/nav2_params.yaml` — added `default_nav_to_pose_bt_xml` parameter
- `params/fastdds_profile.xml` — new file for container DDS compatibility
- `setup.py` — updated to install XML params files

### Clean Up Stale project/ Directory
`/workspaces/ros2_ws/project/` contains old copies of both packages with a `COLCON_IGNORE` marker. This directory should be deleted to avoid confusion.

### Make CrowdStop proximity_distance Configurable
Currently `proximity_distance_` is hardcoded to 1.0m in `crowd_stop.cpp`. Adding it as a BT input port would allow tuning per-environment:

```xml
<CrowdStop density_threshold="0.5" proximity_distance="1.5"/>
```

Requires changes to:
- `crowd_stop.hpp` — remove hardcoded value
- `crowd_stop.cpp` — read from input port in constructor, add to `providedPorts()`
