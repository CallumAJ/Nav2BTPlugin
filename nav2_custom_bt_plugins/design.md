| Plugin           | Type      | Inputs            | Output          |
| ---------------- | --------- | ----------------- | --------------- |
| BatteryMonitor   | Condition | threshold         | true if low     |
| ObstacleSlowdown | Decorator | distance          | speed scale     |
| CrowdStop        | Condition | density threshold | stop signal     |
| ReturnToDock     | Action    | dock_pose         | navigation goal |
