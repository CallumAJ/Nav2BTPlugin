# TODO

## Groot2
- Download and install Groot2
- Load `custom_nav_tree.xml` and verify all 4 custom nodes render
- Export a visual BT diagram for the report and poster

## Experimental Metrics
- Run baseline trials with default Nav2 (no custom plugins)
- Run trials with custom BT plugins enabled
- Collect metrics: navigation time, speed profiles near obstacles, battery-triggered dock returns, crowd stop activations
- Generate comparison plots (baseline vs custom system)

## Final Report
- Introduction and motivation
- System architecture and BT design (include Groot2 diagram)
- Plugin descriptions and implementation details
- Experimental setup and results (baseline vs custom)
- Discussion, limitations, and future work

## Poster / Presentation
- System diagram showing the BT structure
- Key results and comparison plots
- Demo screenshots or video from Gazebo/RViz

## Demo
- Prepare a live or recorded demo showing:
  - Normal navigation with obstacle slowdown
  - Crowd stop triggering
  - Low battery dock return

## Code Submission
- Clean up repos
- Ensure build and test instructions work from scratch

## Resolved
- ~~Speed Limit May Not Affect DWB Controller~~ — Verified working
- ~~Make CrowdStop proximity_distance Configurable~~ — Implemented as BT input port
- ~~Clean Up Stale project/ Directory~~ — Deleted
- ~~Add Unit Tests~~ — 11 gtest-based tests, all passing
- ~~CrowdStop Is Hard to Trigger~~ — Tuned thresholds
- ~~Push nav2_bt_project Changes~~ — Pushed
- ~~Battery Simulator Crashes on Relaunch~~ — Added retry logic
