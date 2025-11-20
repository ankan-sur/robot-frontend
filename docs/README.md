# HFH Robot UI

Web UI for Henry Ford Health robot, built with React/TypeScript and connected directly to ROS 2 via rosbridge.

What’s included
- Frontend (`frontend/`): React + TypeScript + Vite, connects to ROS 2 via roslib.js over rosbridge_websocket.
- ROS integration: Uses rosbridge_server (port 9090), web_video_server (port 8080), and rosboard (port 8888).
- Optional: A minimal Python backend exists but is not required for normal operation.

Quick start (development)
- Robot: Ensure ROS 2 bringup is running with rosbridge_websocket, web_video_server, rosboard, and your system_topics node.
- UI:
  - `cd frontend && npm install`
  - Dev server: `npm run dev -- --host`
  - Open: `http://fordward.local:5173` (or `http://<robot-ip>:5173`)

Production serve options
- Build: `cd frontend && npm run build`
- Preview locally: `npm run preview -- --host` → `http://<host>:4173`
- Nginx on the robot: follow `docs/HOSTNAME_AND_SERVE.md` (serve `frontend/dist` at `http://fordward.local`).

Robot-side prerequisites
- ROS 2 (Humble) stack with:
  - rosbridge_server (9090)
  - web_video_server (8080)
  - rosboard (8888)
  - system_topics node publishing UI/system topics (e.g., /map, /pois, /robot/state)
- mDNS (Avahi) so `fordward.local` resolves on the LAN.

Docs
- **`docs/CURRENT_STATE.md`** – **START HERE**: Latest updates, bug fixes, deployment instructions, testing checklist
- `docs/ARCHITECTURE.md` – how pieces fit (rosbridge + WVS + rosboard + system_topics)
- `docs/HOSTNAME_AND_SERVE.md` – serve the UI at `fordward.local`
- `docs/RASPBERRY_PI_SETUP.md` – configure a fresh robot (ROS 2 + rosbridge + WVS + rosboard)
- `docs/CONNECTION_GUIDE.md` – end-to-end connect instructions
- `docs/ROBOT_SETUP.md` – quick robot bringup checklist
- `docs/FRONTEND_DOCUMENTATION.txt` – UI components and ROS hooks

Common URLs
- UI dev: `http://fordward.local:5173`
- UI prod (preview): `http://fordward.local:4173`
- Camera: `http://fordward.local:8080/stream?topic=/ascamera/camera_publisher/rgb0/image&type=mjpeg`
- Rosboard: `http://fordward.local:8888`

Notes
- The UI auto-targets the current host for rosbridge/video/rosboard; override via `frontend/.env` if ports differ.
