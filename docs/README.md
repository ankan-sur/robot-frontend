# HFH Robot Frontend# HFH Robot UI



Web-based control interface for Henry Ford Health robot. Built with React/TypeScript, connects directly to ROS 2 via rosbridge.Web UI for Henry Ford Health robot, built with React/TypeScript and connected directly to ROS 2 via rosbridge.



## Quick StartWhat’s included

- Frontend (`frontend/`): React + TypeScript + Vite, connects to ROS 2 via roslib.js over rosbridge_websocket.

### Prerequisites- ROS integration: Uses rosbridge_server (port 9090), web_video_server (port 8080), and rosboard (port 8888).

- Robot running ROS 2 with rosbridge_websocket (port 9090)- Optional: A minimal Python backend exists but is not required for normal operation.

- web_video_server (port 8080) for camera feed

- Node.js 18+ for developmentQuick start (development)

- Robot: Ensure ROS 2 bringup is running with rosbridge_websocket, web_video_server, rosboard, and your system_topics node.

### Development- UI:

```bash  - `cd frontend && npm install`

cd frontend  - Dev server: `npm run dev -- --host`

npm install  - Open: `http://fordward.local:5173` (or `http://<robot-ip>:5173`)

npm run dev -- --host

```Production serve options

Open `http://localhost:5173` or `http://<robot-ip>:5173`- Build: `cd frontend && npm run build`

- Preview locally: `npm run preview -- --host` → `http://<host>:4173`

### Production Build- Nginx on the robot: follow `docs/HOSTNAME_AND_SERVE.md` (serve `frontend/dist` at `http://fordward.local`).

```bash

cd frontendRobot-side prerequisites

npm run build- ROS 2 (Humble) stack with:

npm run preview -- --host  # Preview at port 4173  - rosbridge_server (9090)

```  - web_video_server (8080)

  - rosboard (8888)

## Architecture  - system_topics node publishing UI/system topics (e.g., /map, /pois, /robot/state)

- mDNS (Avahi) so `fordward.local` resolves on the LAN.

```

┌─────────────────────────────────────────────┐Docs

│ Robot (ROS 2)                               │- **`docs/CURRENT_STATE.md`** – **START HERE**: Latest updates, bug fixes, deployment instructions, testing checklist

│  ├─ rosbridge_websocket (WS 9090)           │- `docs/ARCHITECTURE.md` – how pieces fit (rosbridge + WVS + rosboard + system_topics)

│  ├─ web_video_server (HTTP 8080)            │- `docs/HOSTNAME_AND_SERVE.md` – serve the UI at `fordward.local`

│  └─ mode_manager, system_topics, etc.       │- `docs/RASPBERRY_PI_SETUP.md` – configure a fresh robot (ROS 2 + rosbridge + WVS + rosboard)

└───────────────↑─────────────────────────────┘- `docs/CONNECTION_GUIDE.md` – end-to-end connect instructions

                │ roslib.js over WebSocket- `docs/ROBOT_SETUP.md` – quick robot bringup checklist

┌───────────────┴─────────────────────────────┐- `docs/FRONTEND_DOCUMENTATION.txt` – UI components and ROS hooks

│ Frontend (React + TypeScript + Vite)        │

│  ├─ App.tsx (main UI)                       │Common URLs

│  ├─ components/ (MapView, TeleopBlock, etc) │- UI dev: `http://fordward.local:5173`

│  └─ ros/ (hooks, services, config)          │- UI prod (preview): `http://fordward.local:4173`

└─────────────────────────────────────────────┘- Camera: `http://fordward.local:8080/stream?topic=/ascamera/camera_publisher/rgb0/image&type=mjpeg`

```- Rosboard: `http://fordward.local:8888`



## FeaturesNotes

- The UI auto-targets the current host for rosbridge/video/rosboard; override via `frontend/.env` if ports differ.

- **Mode Control**: Switch between Idle, SLAM (mapping), and Navigation modes
- **Map Management**: Save and load maps
- **Teleop**: Keyboard (WASD) and touch joystick control
- **Live Map**: Real-time occupancy grid with robot pose
- **Camera Feed**: MJPEG stream from robot camera
- **Battery Monitor**: Live battery percentage and voltage

## Project Structure

```
frontend/
├── src/
│   ├── App.tsx              # Main application
│   ├── components/
│   │   ├── MapView.tsx      # Map visualization
│   │   ├── TeleopBlock.tsx  # Teleop controls
│   │   ├── VideoFeed.tsx    # Camera stream
│   │   ├── VirtualJoystick.tsx  # Touch joystick
│   │   └── DebugLog.tsx     # ROS debug logs
│   └── ros/
│       ├── config.ts        # Topic/service config
│       ├── hooks.ts         # React hooks for ROS
│       ├── services.ts      # ROS service calls
│       └── ros.ts           # Connection manager
└── package.json
```

## Documentation

- **[ARCHITECTURE.md](./ARCHITECTURE.md)** - System design and data flow
- **[FRONTEND_GUIDE.md](./FRONTEND_GUIDE.md)** - Component and hook reference
- **[ROS_INTEGRATION.md](./ROS_INTEGRATION.md)** - Topics, services, and ROS API
- **[CONNECTION_GUIDE.md](./CONNECTION_GUIDE.md)** - Connecting to the robot
- **[ROBOT_SETUP.md](./ROBOT_SETUP.md)** - Robot bringup checklist
- **[RASPBERRY_PI_SETUP.md](./RASPBERRY_PI_SETUP.md)** - Fresh Pi setup
- **[HOSTNAME_AND_SERVE.md](./HOSTNAME_AND_SERVE.md)** - Production deployment
- **[GIT_SETUP.md](./GIT_SETUP.md)** - Git configuration

## Common URLs

| Service | URL |
|---------|-----|
| UI (dev) | `http://fordward.local:5173` |
| UI (prod) | `http://fordward.local:4173` |
| Camera | `http://fordward.local:8080/stream?topic=/ascamera/camera_publisher/rgb0/image&type=mjpeg` |
| Rosboard | `http://fordward.local:8888` |

## Environment Variables (Optional)

Create `frontend/.env` to override defaults:
```env
VITE_ROSBRIDGE_URL=ws://192.168.1.100:9090
VITE_VIDEO_BASE=http://192.168.1.100:8080
VITE_CAMERA_TOPIC=/camera/image
```

By default, the UI auto-detects the host from the browser URL.
