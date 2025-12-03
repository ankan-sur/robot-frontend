# Frontend Integration Guide - Cloud Mode

Guide for integrating cloud WebSocket client into the existing React app.

## Overview

The cloud mode adds an alternative connection method to the existing direct rosbridge connection. You can switch between modes using environment variables.

## Architecture

### Before (Direct Mode)
```
Browser → roslib.js → ws://robot.local:9090 (rosbridge)
```

### After (Cloud Mode)
```
Browser → cloudClient.ts → wss://backend.com/ui → backend → ws://robot.com/robot
```

## Integration Steps

### 1. Environment Configuration

Create `.env.production` for cloud deployment:

```bash
# Cloud mode (production)
VITE_USE_CLOUD_MODE=true
VITE_CLOUD_BACKEND_URL=wss://your-app.onrender.com/ui
VITE_ROBOT_ID=mentorpi
```

Keep `.env.local` for local development:

```bash
# Direct mode (development)
VITE_USE_CLOUD_MODE=false
VITE_ROSBRIDGE_URL=ws://fordward.local:9090
VITE_VIDEO_BASE=http://fordward.local:8080
```

### 2. Update App.tsx

Add cloud mode detection at the top of your main App component:

```typescript
import { useCloudConnection } from './hooks/useCloudConnection';
import { useROS } from './hooks/useROS'; // Your existing ROS hook

function App() {
  const useCloudMode = import.meta.env.VITE_USE_CLOUD_MODE === 'true';
  
  // Use cloud connection or direct ROS connection
  const cloudConnection = useCloudConnection();
  const rosConnection = useROS();
  
  const connection = useCloudMode ? {
    connected: cloudConnection.connected,
    error: cloudConnection.error,
    // Map cloud state to your existing interface
    state: cloudConnection.robotState.state,
    battery: cloudConnection.robotState.battery,
    pose: cloudConnection.robotState.pose,
    // ... other mappings
  } : rosConnection;
  
  // Rest of your app uses `connection` which works the same way
  // whether it's cloud or direct mode
}
```

### 3. Command Mapping

Map your existing command functions to cloud commands:

```typescript
const startSlam = useCloudMode 
  ? cloudConnection.startSlam 
  : () => callRosService('/start_slam');

const loadMap = useCloudMode
  ? cloudConnection.loadMap
  : (mapName: string) => callRosService('/load_map', { data: mapName });

// Use these wrapped functions in your UI
<button onClick={startSlam}>Start Mapping</button>
<button onClick={() => loadMap('map1')}>Load Map</button>
```

### 4. Create Compatibility Layer (Recommended)

Create a unified interface that works with both modes:

```typescript
// src/hooks/useRobotConnection.ts

import { useCloudConnection } from './useCloudConnection';
import { useROS } from './useROS';

export interface UnifiedRobotConnection {
  connected: boolean;
  error: string | null;
  state: string;
  battery: number | null;
  pose: any;
  availableMaps: string[];
  
  // Commands
  startSlam: () => void;
  stopSlam: (mapName: string) => void;
  loadMap: (mapName: string) => void;
  setMode: (mode: string) => void;
  cancelNavigation: () => void;
  emergencyStop: () => void;
}

export function useRobotConnection(): UnifiedRobotConnection {
  const useCloudMode = import.meta.env.VITE_USE_CLOUD_MODE === 'true';
  
  const cloud = useCloudConnection();
  const ros = useROS();
  
  if (useCloudMode) {
    // Cloud mode
    return {
      connected: cloud.connected,
      error: cloud.error,
      state: cloud.robotState.state,
      battery: cloud.robotState.battery,
      pose: cloud.robotState.pose,
      availableMaps: cloud.robotState.availableMaps,
      
      startSlam: cloud.startSlam,
      stopSlam: cloud.stopSlam,
      loadMap: cloud.loadMap,
      setMode: cloud.setMode,
      cancelNavigation: cloud.cancelNavigation,
      emergencyStop: cloud.emergencyStop,
    };
  } else {
    // Direct ROS mode
    return {
      connected: ros.connected,
      error: ros.error,
      state: ros.state,
      battery: ros.battery,
      pose: ros.pose,
      availableMaps: ros.availableMaps,
      
      startSlam: () => ros.callService('/start_slam'),
      stopSlam: (mapName) => ros.callService('/stop_slam_and_save', { data: mapName }),
      loadMap: (mapName) => ros.callService('/load_map', { data: mapName }),
      setMode: (mode) => ros.callService('/set_mode', { data: mode }),
      cancelNavigation: () => ros.callService('/navigate_to_pose/cancel'),
      emergencyStop: () => ros.publishCmdVel({ linear: { x: 0 }, angular: { z: 0 } }),
    };
  }
}
```

### 5. Update Components

Now your components use the unified interface:

```typescript
function RobotControls() {
  const robot = useRobotConnection();
  
  return (
    <div>
      <p>Status: {robot.connected ? 'Connected' : 'Disconnected'}</p>
      <p>State: {robot.state}</p>
      <p>Battery: {robot.battery}%</p>
      
      <button onClick={robot.startSlam}>Start SLAM</button>
      <button onClick={robot.cancelNavigation}>Cancel</button>
      <button onClick={robot.emergencyStop}>STOP</button>
      
      <select onChange={(e) => robot.loadMap(e.target.value)}>
        {robot.availableMaps.map(map => (
          <option key={map} value={map}>{map}</option>
        ))}
      </select>
    </div>
  );
}
```

## Video Streaming

### Cloud Mode
Video streaming is **not included** in cloud mode by default (high bandwidth).

**Options:**
1. Keep direct connection to robot for video (use local IP)
2. Use separate streaming service (RTSP, HLS, WebRTC)
3. Stream through backend (not recommended - expensive bandwidth)

**Recommended approach:**

```typescript
const videoUrl = useCloudMode
  ? `https://robot-stream.example.com/stream` // Separate streaming service
  : `http://${robotHost}:8080/stream?topic=/camera/image`; // Direct connection

<img src={videoUrl} alt="Robot camera" />
```

## Testing Both Modes

### Test Direct Mode (Local)

```bash
cd frontend
cp .env.example .env.local

# Edit .env.local
VITE_USE_CLOUD_MODE=false
VITE_ROSBRIDGE_URL=ws://fordward.local:9090

npm run dev
```

### Test Cloud Mode (Production)

```bash
# Edit .env.production
VITE_USE_CLOUD_MODE=true
VITE_CLOUD_BACKEND_URL=wss://your-app.onrender.com/ui

npm run build
npm run preview
```

## Deployment

### Development (Direct Mode)
```bash
npm run dev
# Connect directly to robot on local network
```

### Production (Cloud Mode)
```bash
# Set environment variables in Vercel
VITE_USE_CLOUD_MODE=true
VITE_CLOUD_BACKEND_URL=wss://your-app.onrender.com/ui

# Deploy
vercel --prod
```

## Feature Comparison

| Feature | Direct Mode | Cloud Mode |
|---------|-------------|------------|
| **Connection** | Direct to robot | Via cloud backend |
| **Network** | Same LAN required | Works anywhere |
| **Latency** | ~10-50ms | ~100-300ms |
| **Video** | Full quality | Not included* |
| **Commands** | Direct ROS calls | Relayed through cloud |
| **Setup** | Simple | Requires backend deploy |
| **Firewall** | Must access robot | Robot connects out |

\* Video can be added via separate streaming service

## Migration Checklist

- [ ] Create `cloudClient.ts` and `useCloudConnection.ts`
- [ ] Add environment variable detection
- [ ] Create unified `useRobotConnection` hook
- [ ] Update all components to use unified hook
- [ ] Test direct mode locally
- [ ] Deploy backend to Render.com
- [ ] Configure cloud bridge on robot
- [ ] Test cloud mode with production backend
- [ ] Deploy frontend to Vercel with cloud mode enabled
- [ ] Verify end-to-end functionality

## Troubleshooting

### "Cannot find module cloudClient"
- Check file paths in imports
- Ensure `cloudClient.ts` is in `src/ros/` directory

### Environment variables not working
- Restart dev server after changing `.env` files
- Check variable names start with `VITE_`
- Use `import.meta.env.VITE_*` not `process.env.*`

### Both modes fail
- Check browser console for detailed errors
- Verify WebSocket URLs are correct
- Test backend health endpoint

### Cloud mode works but direct mode breaks
- Keep both code paths working
- Don't remove roslib.js or existing ROS code
- Use conditional logic: `if (useCloudMode) { ... } else { ... }`

## Example: Full Integration

```typescript
// src/App.tsx

import { useRobotConnection } from './hooks/useRobotConnection';

function App() {
  const robot = useRobotConnection();
  
  if (!robot.connected) {
    return <div>Connecting to robot...</div>;
  }
  
  return (
    <div className="app">
      <header>
        <h1>Robot Control</h1>
        <div className="status">
          <span>State: {robot.state}</span>
          <span>Battery: {robot.battery}%</span>
        </div>
      </header>
      
      <main>
        <section className="controls">
          <button onClick={robot.startSlam}>
            Start Mapping
          </button>
          
          <select onChange={(e) => robot.loadMap(e.target.value)}>
            <option>Select map...</option>
            {robot.availableMaps.map(map => (
              <option key={map} value={map}>{map}</option>
            ))}
          </select>
          
          <button 
            onClick={robot.emergencyStop}
            className="emergency"
          >
            EMERGENCY STOP
          </button>
        </section>
        
        <section className="map">
          {/* Map visualization */}
        </section>
      </main>
    </div>
  );
}

export default App;
```

## Best Practices

1. **Keep both modes working** - Don't break direct mode when adding cloud mode
2. **Use environment variables** - Easy switching between modes
3. **Unified interface** - Components shouldn't care about connection type
4. **Error handling** - Show clear messages for connection issues
5. **Loading states** - Display connection status to users
6. **Fallback** - Consider direct mode fallback if cloud fails

## Next Steps

1. Implement unified hook as shown above
2. Test with both backends
3. Add connection status UI
4. Add command result feedback
5. Deploy and test in production

---

See `cloudClient.ts` and `useCloudConnection.ts` for implementation details.
