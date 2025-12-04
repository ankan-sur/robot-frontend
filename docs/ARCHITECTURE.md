# Architecture

## System Overview

The HFH Robot Dashboard is a browser-based UI that communicates directly with ROS 2 via rosbridge WebSocket. No backend server is required.

```
┌─────────────────────────────────────────────────┐
│ Robot (Raspberry Pi / ROS 2 Humble)             │
│                                                 │
│  ┌─────────────────────────────────────────┐    │
│  │ Core ROS 2 Nodes                        │    │
│  │  • mode_manager (SLAM/Nav switching)    │    │
│  │  • system_topics (maps, POIs, state)    │    │
│  │  • teleop_gateway (cmd_vel routing)     │    │
│  │  • ros_robot_controller (battery, etc)  │    │
│  └─────────────────────────────────────────┘    │
│                                                 │
│  ┌─────────────────────────────────────────┐    │
│  │ Bridge Services                         │    │
│  │  • rosbridge_websocket (port 9090)      │    │
│  │  • web_video_server (port 8080)         │    │
│  │  • rosboard (port 8888, optional)       │    │
│  └─────────────────────────────────────────┘    │
└───────────────────────┬─────────────────────────┘
                        │
          WebSocket (roslib.js) + HTTP (video)
                        │
┌───────────────────────┴─────────────────────────┐
│ Frontend (Browser)                              │
│                                                 │
│  ┌─────────────────────────────────────────┐    │
│  │ React Application                       │    │
│  │  • App.tsx - Main layout & state        │    │
│  │  • MapView - Occupancy grid + pose      │    │
│  │  • TeleopBlock - Keyboard/joystick      │    │
│  │  • VideoFeed - Camera stream            │    │
│  │  • DebugLog - ROS console output        │    │
│  └─────────────────────────────────────────┘    │
│                                                 │
│  ┌─────────────────────────────────────────┐    │
│  │ ROS Integration Layer                   │    │
│  │  • ros.ts - Connection management       │    │
│  │  • config.ts - Topic/service names      │    │
│  │  • hooks.ts - React hooks for topics    │    │
│  │  • services.ts - Service call helpers   │    │
│  └─────────────────────────────────────────┘    │
└─────────────────────────────────────────────────┘
```

## Data Flow

### Subscriptions (Robot → UI)

| Data | Topic | Type | Rate |
|------|-------|------|------|
| Robot pose | `/amcl_pose` or `/robot/pose` | PoseWithCovarianceStamped | 10 Hz |
| Battery | `/ros_robot_controller/battery` | UInt16 (millivolts) | 0.5 Hz |
| Map | `/map` | OccupancyGrid | On change |
| Mode status | via `/get_mode` service | JSON | On request |
| Available maps | via `/list_maps` service | JSON array | On request |
| Debug logs | `/rosout` | Log | Real-time |

### Publications (UI → Robot)

| Command | Topic | Type |
|---------|-------|------|
| Teleop velocity | `/ui/cmd_vel` | Twist |

### Services (UI → Robot)

| Action | Service | Type |
|--------|---------|------|
| Get current mode | `/get_mode` | Trigger |
| Set mode | `/set_mode` | SetString |
| Save map | `/stop_slam_and_save` | SetString |
| Load map | `/load_map` | SetString |
| List maps | `/list_maps` | Trigger |
| Cancel navigation | `/navigate_to_pose/cancel` | CancelGoal |

## Component Architecture

```
App.tsx
├── Header (connection status, battery, mode indicator)
├── Main Content
│   ├── Left Column (lg:col-span-2)
│   │   ├── Map/Camera Tabs
│   │   │   ├── MapView (occupancy grid + robot pose)
│   │   │   └── VideoFeed (MJPEG camera stream)
│   │   ├── Mode Control Panel
│   │   │   ├── Idle/SLAM/Nav buttons
│   │   │   └── Map selection (for Nav mode)
│   │   └── DebugLog (collapsible)
│   └── Right Column
│       ├── TeleopBlock
│       │   ├── Keyboard controls (WASD)
│       │   └── VirtualJoystick (touch)
│       └── Settings Panel (collapsible)
└── Save Map Dialog (modal)
```

## State Management

All ROS data flows through custom React hooks:

- **`useRosConnection()`** - Connection state and reconnection
- **`useModeAndMaps()`** - Current mode, active map, available maps
- **`useRobotPose()`** - Robot position from AMCL or odom
- **`useBattery()`** - Battery voltage and percentage
- **`useMap()`** - Occupancy grid data
- **`useCmdVel()`** - Velocity command publisher

Local UI state is managed with React's `useState`:
- `operating` - Loading state during service calls
- `statusMsg` - Success/error messages
- `selectedMap` - User's map selection
- `activeTab` - Map vs Camera tab
- `showSettings` / `showDebugLog` - Panel visibility

## Configuration

All ROS topic and service names are centralized in `ros/config.ts`:

```typescript
export const ROS_CONFIG = {
  rosbridgeUrl: 'ws://<host>:9090',  // Auto-detected from browser
  videoBase: 'http://<host>:8080',   // Auto-detected from browser
  topics: {
    cmdVel: '/ui/cmd_vel',
    battery: '/ros_robot_controller/battery',
    map: '/map',
    // ...
  },
  services: {
    setMode: '/set_mode',
    loadMap: '/load_map',
    // ...
  }
}
```

## Operating Modes

### Idle Mode
- No SLAM or navigation running
- Can view saved maps (read-only)
- Camera feed available
- Teleop available

### SLAM Mode
- slam_toolbox running in sync mode
- Live map building displayed
- Teleop for manual exploration
- "Save Map & Stop" to save and return to idle

### Navigation Mode (Localization)
- AMCL running with loaded map
- Robot pose tracking on map
- Teleop available
- Future: autonomous navigation goals

## Network Ports

| Port | Service | Protocol |
|------|---------|----------|
| 9090 | rosbridge_websocket | WebSocket |
| 8080 | web_video_server | HTTP/MJPEG |
| 8888 | rosboard | HTTP (optional) |
| 5173 | Vite dev server | HTTP |
| 4173 | Vite preview | HTTP |
