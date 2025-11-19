# Integration Audit — Frontend ↔ Robot (generated)

Date: 2025-11-19

This file maps frontend expectations (topics, services, actions) to robot-side providers (packages, nodes, launch files), plus notes and recommended fixes.

## Summary (high level)
- Frontend: React + TypeScript + Vite connecting to ROS 2 via rosbridge (roslib.js). Config lives in `frontend/src/ros/config.ts` and hooks in `frontend/src/ros/hooks.ts`.
- Robot: ROS 2 workspace `robothome/` contains `bringup` (launch files), `system_topics`, `teleop_gateway`, `mode_manager`, `ros_robot_controller`, SLAM, Navigation, etc.
- Most expected topics/services are present and brought up by `core_bringup.launch.py`.

## Changes applied during audit
- Frontend `ROS_CONFIG` updated to use canonical `/robot/state` (was `/robot_state`) in `frontend/src/ros/config.ts`.
- `useRobotState()` in `frontend/src/ros/hooks.ts` updated to subscribe to both `/robot/state` and fallback `/robot_state` for resilience.

## Per-contract checklist

| Frontend name | Message/Service type | Robot provider (file / note) | Present | Notes / Action required |
|---|---:|---|---:|---|
| rosbridge (ws://:9090) | WebSocket JSON | `robothome/src/bringup/launch/core_bringup.launch.py` (rosbridge_websocket) | ✅ | Ensure port 9090 reachable from client.
| web_video_server (http :8080) | MJPEG HTTP | `core_bringup.launch.py` / `web_video_server` Node | ✅ | Streams at `/stream?topic=/ascamera/...`.
| rosboard (http :8888) | HTTP UI | `bringup.launch.py` (conditional) | ✅ (optional) | Set `use_rosboard` true to enable.
| /ui/cmd_vel | geometry_msgs/Twist | `teleop_gateway` subscribes to `/ui/cmd_vel` -> `/app/cmd_vel` (`robothome/src/teleop_gateway/...`) | ✅ | Frontend uses `/ui/cmd_vel` by default.
| /app/cmd_vel | geometry_msgs/Twist | Output of `teleop_gateway` | ✅ | Controllers expect downstream topic `/app/cmd_vel` or further remap.
| /odom | nav_msgs/Odometry | controller/SLAM/navigation configs (many files) | ✅ | Published by controller stack.
| /ros_robot_controller/battery | std_msgs/UInt16 | `ros_robot_controller_node.py` publishes `~/battery` → `/ros_robot_controller/battery` | ✅ | Frontend `useBattery()` expects millivolts.
| /ascamera/.../image | sensor_msgs/Image | depth camera drivers (ascamera) | ✅ | UI uses WVS for streaming.
| /map | nav_msgs/OccupancyGrid | map_server / nav2 / SLAM (when running) | Conditional | `system_topics` can publish static map if configured; SLAM/Nav2 publishes real map.
| /map_metadata | nav_msgs/MapMetaData | SLAM/Nav2 / map_server | Conditional | Present when map server active.
| /available_maps | std_msgs/String | `system_topics` publishes JSON array | ✅ | Frontend consumes this.
| /connected_clients, /client_count | std_msgs/String / Int32 | `system_topics` | ✅ | Present.
| /system/map/info | interfaces/SetString (JSON) | `system_topics` service | ✅ | Frontend prefers this service for POIs.
| /system/map/pois | interfaces/SetString | `system_topics` service | ✅ | Fallbacks included in frontend.
| /navigate_to_pose (action) | nav2_msgs/action/NavigateToPose | Nav2 (when running via mode_manager) | Conditional | UI action client present; requires nav2.
| /navigate_to_pose/status | action_msgs/GoalStatusArray | `system_topics` publishes empty stub by default if enabled | ✅ (stub) | Real statuses supplied by Nav2 when active.
| /navigate_to_pose/cancel | action_msgs/CancelGoal | Provided by Nav2 or `system_topics` stub if configured | ✅ (stub) | `system_topics` can create a stub to accept cancels.
| Mode services (`/get_mode`,`/set_mode`,`/start_slam`, `/stop_slam_and_save`,`/load_map`,`/list_maps`) | std_srvs/Trigger / interfaces/SetString | `mode_manager` package in bringup | ✅ | Frontend uses service-based mode handling; ensure `mode_manager` is launched.
| /robot/state (canonical) | std_msgs/String | `system_topics` publishes `/robot/state` | ✅ | Frontend previously used `/robot_state`; updated to `/robot/state` and hook now falls back to legacy name.


## Noted mismatches / recommendations
- Robot state topic name: frontend previously read `/robot_state`. This is fixed; the frontend now prefers `/robot/state` and subscribes to both.
- Map and navigation behavior: ensure `mode_manager` loads SLAM/Nav2 for navigation features. Without SLAM/Nav2, map and navigation action won't be available.
- Battery topic conversion: frontend expects millivolts in `std_msgs/UInt16` — confirm `ros_robot_controller` publishes millivolts (it does in code via UInt16).

## Suggested tests (smoke checks)
Run on robot:

- Start bringup (development):

```bash
ros2 launch bringup core_bringup.launch.py
```

- Verify topics/services:

```bash
ros2 topic list | grep -E "available_maps|robot/state|navigate_to_pose|/map|ascamera"
ros2 topic echo /available_maps --once
ros2 topic echo /robot/state --once
ros2 topic info /ros_robot_controller/battery
ros2 service list | grep -E "system/map|get_mode|list_maps"
```

From frontend machine:
- Start dev server: `cd frontend && npm install && npm run dev -- --host` and open the UI. Verify connection, map, POIs, and camera stream.

## Next steps I can take (pick one)
- Implement a small diagnostics panel in the UI that lists reachable topics/services via rosbridge.
- Add tests for `useBattery()` message parsing and for `useRobotState()` fallback behavior.

---
Generated by in-repo audit on 2025-11-19.
