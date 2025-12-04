# Frontend Guide

## Components

The frontend has 5 active components in `frontend/src/components/`:

### MapView.tsx

Renders the occupancy grid map with robot pose overlay.

**Props:**
- `embedded?: boolean` - Render without card wrapper
- `mode?: string` - Current operating mode (for display context)

**Features:**
- Canvas-based occupancy grid rendering
- Robot pose marker (arrow showing position and heading)
- Auto-scales to fit container
- Color scheme: gray (unknown), white (free), black (occupied)

### TeleopBlock.tsx

Teleoperating controls with keyboard and touch support.

**Props:**
- `disableKeyboard?: boolean` - Disable keyboard input (e.g., when modal is open)

**Features:**
- WASD/Arrow key support for desktop
- Virtual joystick for mobile/touch
- Configurable speed limits
- Stop button for immediate halt

### VideoFeed.tsx

Camera stream display using web_video_server.

**Props:**
- `embedded?: boolean` - Render without card wrapper

**Features:**
- MJPEG stream from web_video_server
- Auto-reconnect on failure
- Placeholder when disconnected

### VirtualJoystick.tsx

Touch-based joystick for mobile teleop.

**Props:**
- `onMove: (linear: number, angular: number) => void` - Movement callback
- `onStop: () => void` - Stop callback

**Features:**
- Touch drag for directional control
- Visual feedback ring
- Auto-centers on release

### DebugLog.tsx

ROS console output display.

**Features:**
- Subscribes to `/rosout` topic
- Color-coded by log level
- Auto-scroll with manual pause
- Collapsible panel

## React Hooks

All ROS integration is through custom hooks in `frontend/src/ros/hooks.ts`:

### Connection

```typescript
const { connected, state, latency } = useRosConnection()
// connected: boolean
// state: 'disconnected' | 'connecting' | 'connected'
// latency: number | null (ms)
```

### Mode and Maps

```typescript
const { mode, activeMap, maps, loading, error, refresh } = useModeAndMaps()
// mode: 'idle' | 'slam' | 'localization' | null
// activeMap: string | null
// maps: string[]
// refresh: () => void - manually refresh data
```

### Robot Pose

```typescript
const pose = useRobotPose()
// pose.pose.position: { x, y, z }
// pose.pose.orientation: { x, y, z, w } (quaternion)
```

### Battery

```typescript
const battery = useBattery()
// battery.millivolts: number
// battery.volts: number
// battery.percent: number (0-100)
```

### Teleop

```typescript
const { send, stop } = useCmdVel()
// send(linearX, angularZ, maxLinear?, maxAngular?)
// stop() - sends zero velocity
```

### Map Data

```typescript
const map = useMap()
// map.info: { resolution, width, height, origin }
// map.data: number[] - occupancy values
```

## Services

Service calls are in `frontend/src/ros/services.ts`:

```typescript
import { setMode, loadMap, stopSlamAndSave, listMaps, getMode } from './ros/services'

// Switch mode
await setMode('slam')        // Start mapping
await setMode('localization') // Start navigation
await setMode('idle')        // Stop all

// Map operations
await stopSlamAndSave('my_map')  // Save current map
await loadMap('my_map')          // Load saved map
const maps = await listMaps()    // Get available maps

// Get current state
const { mode, map } = await getMode()
```

## Configuration

Edit `frontend/src/ros/config.ts` to modify:

- Topic names
- Service names
- Message types
- Default URLs

## Styling

The UI uses Tailwind CSS with a dark theme (slate-800/900 backgrounds).

Key classes:
- `bg-slate-900` - Main background
- `bg-slate-800` - Card backgrounds
- `border-slate-700` - Borders
- `text-slate-300/400` - Secondary text
- Mode colors: amber (SLAM), blue (Nav), slate (Idle)

## Mobile Support

The UI is responsive:
- Touch joystick replaces keyboard on mobile
- Stacked layout on small screens
- Compact battery display on mobile
- Touch-friendly button sizing
