# HFH Robot Architecture

Overview
- Browser UI connects directly to ROS 2 via rosbridge_websocket (no backend required in normal operation).
- Video streams via web_video_server (WVS).
- System/meta topics (maps, POIs, state, clients) are provided by a `system_topics` ROS 2 node.
- Rosboard provides system introspection (graphs, topics, params) for debugging.

Topology
```
┌──────────────────────────────────────────────┐
│ Robot (ROS 2)                                │
│  ├─ bringup (your stack)                     │
│  │   ├─ nav2, tf, sensors, camera, etc.      │
│  │   ├─ system_topics (UI/system feeds)      │
│  │   ├─ rosbridge_websocket (WS 9090)        │
│  │   ├─ web_video_server (HTTP 8080)         │
│  │   └─ rosboard (HTTP 8888)                 │
└───────────────↑──────────────────────────────┘
                │
        roslib.js over WS
                │
┌───────────────┴──────────────────────────────┐
│ Frontend (React + TS + Vite)                 │
│  ├─ connects to ws://<host>:9090             │
│  ├─ renders /map + POIs + robot pose         │
│  ├─ embeds camera via WVS                    │
│  └─ sends Nav2 goals, cancel requests        │
└──────────────────────────────────────────────┘
```

Key topics and services
- Map: `/map` (`nav_msgs/OccupancyGrid`, transient local QoS)
- POIs: `/pois` (`interfaces/Points`, pixel coordinates for overlays)
- Robot state: `/robot/state` (`std_msgs/String`)
- Nav2 status: `/navigate_to_pose/status` (`action_msgs/GoalStatusArray`)
- Clients: `/connected_clients` (`std_msgs/String` JSON), `/client_count` (`std_msgs/Int32`)
- Camera: `/ascamera/camera_publisher/rgb0/image` (`sensor_msgs/Image`) via WVS
- CmdVel: `/cmd_vel` (`geometry_msgs/Twist`) for manual/safety stop
- Cancel navigation service: `/navigate_to_pose/cancel` (`action_msgs/CancelGoal`)

Frontend defaults
- Host autodetect: UI derives `rosbridgeUrl`, `videoBase`, and `rosboard` from `window.location.hostname`.
- See `frontend/src/ros/config.ts:10` for topic names and types.

Coordinate handling (POIs)
- POIs are provided in pixel coordinates matching map image.
- Convert to world coordinates for Nav2 goals using `map.info.origin` and `map.info.resolution`:
  - `x_world = origin.x + (px + 0.5) * resolution`
  - `y_world = origin.y + (height - py - 0.5) * resolution`

Bringup notes
- rosbridge: `rosbridge_server/rosbridge_websocket` on 9090, address `0.0.0.0`.
- WVS: `web_video_server` on 8080.
- Rosboard: `rosboard` on 8888.
- System topics: launch your `system_topics` node with parameters for available maps, connected clients, robot state, and POIs.

Development workflow
- `cd frontend && npm install && npm run dev -- --host`
- Open `http://fordward.local:5173` (or robot IP) to connect.

Troubleshooting
- rosbridge connection: ensure 9090 open; `sudo ufw allow 9090` if needed.
- Video: `http://<host>:8080/stream_viewer?topic=...` should render.
- Rosboard: `http://<host>:8888` loads and lists topics.
- Missing topics: verify `system_topics` and bringup are running; check with `ros2 topic list`.
