# TODO

## Open

- Download Groot2 and load `custom_nav_tree.xml` to verify all 4 custom nodes render correctly

- Export a visual diagram of the behavior tree from Groot2 for the final report/presentation

## Resolved

- ~~Speed Limit May Not Affect DWB Controller~~ — Verified: Nav2's controller_server subscribes to `/speed_limit` and DWB's `setSpeedLimit()` delegates to the trajectory generator. The speed limit chain works correctly.
- ~~Make CrowdStop proximity_distance Configurable~~ — Implemented as a BT input port in `crowd_stop.cpp`.
- ~~Clean Up Stale project/ Directory~~ — Deleted `/workspaces/ros2_ws/project/`. Proposal PDF preserved in Nav2BTPlugin root.
- ~~Add Unit Tests~~ — Added 11 gtest-based tests in `test/test_plugins.cpp` covering all 4 plugins: BatteryMonitor (3 tests), CrowdStop (3 tests), ReturnToDock (2 tests), ObstacleSlowdown (3 tests). All passing.
- ~~CrowdStop Is Hard to Trigger~~ — Lowered `density_threshold` from 0.8 to 0.4 and increased `proximity_distance` from 1.0m to 1.5m in the behavior tree XML for practical testing.
- ~~Push nav2_bt_project Changes~~ — Pushed to https://github.com/anthonytrieu/nav2_bt_project.
- ~~Battery Simulator Crashes on Relaunch~~ — Added retry logic in `battery_simulator.py` that catches `RCLError` and `ExternalShutdownException`, shuts down cleanly, and re-initializes with a fresh context (up to 3 retries).
