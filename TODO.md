# TODO

## Known Issues

### Battery Simulator Crashes on Relaunch
The battery simulator launched via `nav2_bt_bringup.launch.py` crashes with `RCLError: the given context is not valid` when Gazebo restarts into an existing session. The node works correctly when started manually in a separate terminal. The launch-managed instance inherits a stale ROS context from a previous run.

**Workaround:** Start the battery simulator manually after launch:
```bash
ros2 run nav2_bt_project battery_simulator --ros-args \
  -p initial_battery_level:=1.0 -p drain_rate:=0.005 -p use_sim_time:=true
```

## Resolved

- ~~Speed Limit May Not Affect DWB Controller~~ — Verified: Nav2's controller_server subscribes to `/speed_limit` and DWB's `setSpeedLimit()` delegates to the trajectory generator. The speed limit chain works correctly.
- ~~Make CrowdStop proximity_distance Configurable~~ — Implemented as a BT input port in `crowd_stop.cpp`.
- ~~Clean Up Stale project/ Directory~~ — Deleted `/workspaces/ros2_ws/project/`. Proposal PDF preserved in Nav2BTPlugin root.
- ~~Add Unit Tests~~ — Added 11 gtest-based tests in `test/test_plugins.cpp` covering all 4 plugins: BatteryMonitor (3 tests), CrowdStop (3 tests), ReturnToDock (2 tests), ObstacleSlowdown (3 tests). All passing.
- ~~CrowdStop Is Hard to Trigger~~ — Lowered `density_threshold` from 0.8 to 0.4 and increased `proximity_distance` from 1.0m to 1.5m in the behavior tree XML for practical testing.
- ~~Push nav2_bt_project Changes~~ — Pushed to https://github.com/anthonytrieu/nav2_bt_project.
