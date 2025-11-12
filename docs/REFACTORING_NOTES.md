# Refactoring Notes

This document summarizes the transition to a frontend‑only UI that talks directly to ROS 2 via rosbridge_websocket, plus the latest tweaks (POI picker restored and aligned to system topics).

## Summary of Changes

### Frontend Changes

1. **Added roslib dependency** (`package.json`)
   - Added `roslib: ^1.3.0` to dependencies
   - Added `@types/roslib: ^1.0.0` to devDependencies

2. **Created ROS integration layer** (`frontend/src/ros/`)
   - `config.ts` - ROS configuration (topics, URLs, message types)
   - `ros.ts` - ROS connection manager with state management
   - `hooks.ts` - React hooks for ROS topics (useOdom, useBattery, useCmdVel, useRosConnection)

3. **Updated components**
   - `App.tsx` - Uses ROS hooks; passes Nav2 `goToLab` to Controls
   - `VideoFeed.tsx` - Uses web_video_server; sizes to natural image
   - `MapView.tsx` - Renders `/map` + robot pose + POIs overlay
   - `Controls.tsx` - Restored POI dropdown (from `/pois`) and "Go"; Stop publishes zero `cmd_vel`

4. **Configuration**
   - UI autodetects host for rosbridge/video/rosboard; `.env` overrides optional

### Backend

- No backend required for normal operation. A minimal FastAPI stub remains but is not used by the UI.

### Files No Longer Used

- `frontend/src/hooks/useTelemetry.ts` - Replaced by ROS hooks
- `frontend/src/lib/api.ts` - Still exists but not used in App.tsx (may be needed for other features)
- `ros_interface/ros_node.py` - No longer needed (ROS runs in Docker)
- `ros_interface/hiwonder_bridge.py` - May still be needed for hardware, but not for backend API

## Next Steps

1. **Install frontend dependencies**:
   ```bash
   cd frontend
   npm install
   ```

2. **Bringup**:
   - Include rosbridge_websocket (9090), web_video_server (8080), rosboard (8888), and your `system_topics` node in bringup.

3. **Environment**:
   - `.env` optional; defaults target current host. Set if ports differ.

4. **Test battery topic message type**:
   ```bash
   ros2 topic info /ros_robot_controller/battery
   ```
   Update `frontend/src/ros/config.ts` if message type differs from `std_msgs/Float32`

5. **Verify topics are accessible**:
   ```bash
   ros2 topic list
   ros2 topic echo /odom --once
   ros2 topic echo /ros_robot_controller/battery --once
   ```

## Breaking Changes

- Frontend no longer relies on FastAPI or `VITE_API_BASE`.
- Map chooser removed from UI (map still rendered; switching handled externally for now).

## Migration Path

If a backend is ever needed again, prefer `roslibpy` to talk to rosbridge from Python. See `docs/ARCHITECTURE.md`.
