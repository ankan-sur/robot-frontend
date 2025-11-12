# Robot Bringup Quick Checklist

This is a quick reference to get the robot stack up for the UI.

Connect
- Power on the robot.
- Connect to Wi‑Fi hotspot `Fordward` (password `fordward`).
- SSH: `ssh pi@192.168.149.1` (password `fordward`).

Ensure required nodes
- rosbridge_websocket (port 9090, `0.0.0.0`).
- web_video_server (port 8080).
- rosboard (port 8888).
- system_topics (publishes `/map`, `/pois`, `/robot/state`, `/available_maps`, `/connected_clients`, `/client_count`, `/navigate_to_pose/status`; provides `/navigate_to_pose/cancel`).

Verify
- `ros2 topic list` includes the topics above.
- Rosboard: open `http://fordward.local:8888`.
- Camera: `http://fordward.local:8080/stream_viewer?topic=/ascamera/camera_publisher/rgb0/image`.
- Rosbridge open (port only): `nc -zv fordward.local 9090`.

Serve UI (options)
- Development from a laptop: `cd frontend && npm install && npm run dev -- --host`, then open `http://fordward.local:5173`.
- Nginx on the robot (production): see `docs/HOSTNAME_AND_SERVE.md` to serve `frontend/dist` at `http://fordward.local`.

Ports
- 9090: rosbridge
- 8080: web_video_server
- 8888: rosboard
- 80: Nginx (if serving UI)

Firewall (UFW)
- `sudo ufw allow 9090 8080 8888`
