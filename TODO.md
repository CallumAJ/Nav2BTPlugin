# TODO

## Known Issues

### Battery Simulator Crashes on Relaunch
The battery simulator launched via `nav2_bt_bringup.launch.py` crashes with `RCLError: the given context is not valid` when Gazebo restarts into an existing session. The node works correctly when started manually in a separate terminal. The launch-managed instance inherits a stale ROS context from a previous run.

**Workaround:** Start the battery simulator manually after launch:
```bash
ros2 run nav2_bt_project battery_simulator --ros-args \
  -p initial_battery_level:=1.0 -p drain_rate:=0.005 -p use_sim_time:=true
```

### CrowdStop Is Hard to Trigger
The default density threshold of 0.8 requires 80% of laser scan points to be within the proximity distance. This rarely happens in the turtlebot3_world map. The plugin logic is correct but the threshold is too high for practical testing in open environments.

**Possible fixes:**
- Lower `density_threshold` in the behavior tree XML (e.g., 0.4)
- Increase `proximity_distance` in the XML (e.g., 1.5 or 2.0)
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

## Resolved

- ~~Speed Limit May Not Affect DWB Controller~~ — Verified: Nav2's controller_server subscribes to `/speed_limit` and DWB's `setSpeedLimit()` delegates to the trajectory generator. The speed limit chain works correctly.
- ~~Make CrowdStop proximity_distance Configurable~~ — Implemented: `proximity_distance` is now a BT input port in `crowd_stop.cpp` and `providedPorts()`.
- ~~Clean Up Stale project/ Directory~~ — Deleted `/workspaces/ros2_ws/project/`. Proposal PDF preserved in Nav2BTPlugin root.
