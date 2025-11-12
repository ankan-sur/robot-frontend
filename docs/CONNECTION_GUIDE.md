# Robot Connection Guide

This guide explains how to connect the web UI to the robot using rosbridge, and how to verify topics and video.

Overview
- Frontend runs on your computer (or the robot) and connects directly to ROS 2 via rosbridge_websocket.
- Video uses web_video_server (HTTP stream).
- Rosboard is available for diagnostics.

Robot connection info
- SSID: `Fordward`
- Password: `fordward`
- Robot IP (hotspot): `192.168.149.1`
- SSH: `ssh pi@192.168.149.1` (password: `fordward`)

1) Ensure robot bringup is running
- Required nodes on robot:
  - rosbridge_websocket (WS 9090)
  - web_video_server (HTTP 8080)
  - rosboard (HTTP 8888)
  - system_topics (publishes `/map`, `/pois`, `/robot/state`, `/available_maps`, `/connected_clients`, `/client_count`, `/navigate_to_pose/status`)
- Verify:
  - `ros2 topic list`
  - Visit `http://<robot-host>:8888` for rosboard

2) Run the frontend (your computer)
- Install: `cd frontend && npm install`
- Dev server: `npm run dev -- --host`
- Open: `http://fordward.local:5173` (or `http://<robot-ip>:5173`)
- The UI auto-targets the host for rosbridge and video; override via `frontend/.env` if needed.

3) UI features to verify
- Connection badge: shows rosbridge connection status and latency.
- Camera feed: shows `/ascamera/camera_publisher/rgb0/image` via WVS (natural-size image).
- Map view: renders `/map` occupancy grid and overlays POIs and robot pose.
- Controls: POI dropdown from `/pois` (pixel coordinates). “Go” sends a Nav2 goal; “Stop” publishes a zero `cmd_vel`.
- Telemetry: Battery, IMU, button, robot state.

4) Nav2 status and cancel
- Subscribe to `/navigate_to_pose/status` for goal lifecycle.
- Cancel via `/navigate_to_pose/cancel` (supply latest goal_id; if none, cancel all).

Troubleshooting
- rosbridge not reachable: open port 9090; `sudo ufw allow 9090`.
- Video blank: open `http://<host>:8080/stream_viewer?topic=/ascamera/camera_publisher/rgb0/image` to test.
- Map missing: ensure `/map` is published (transient local); verify with `ros2 topic echo /map`.
- POIs missing: ensure `/pois` is published by system_topics (or your source); see rosboard.

Notes
- Namespacing: current topics are at root; if namespaced later, update `frontend/src/ros/config.ts`.
- Pixel-to-world conversion happens when sending Nav2 goals (based on `map.info`).
