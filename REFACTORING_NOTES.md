# Refactoring Notes

This document describes the changes made to refactor the architecture to use rosbridge_websocket.

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
   - `App.tsx` - Now uses ROS hooks instead of FastAPI WebSocket
   - `VideoFeed.tsx` - Uses web_video_server instead of backend endpoint
   - `MapView.tsx` - Displays robot position from odometry
   - `Controls.tsx` - Updated to use ROS cmd_vel directly

4. **Updated environment configuration** (`env.example`)
   - Changed from API base URL to ROS bridge and video server URLs
   - Uses `fordward.local` hostname instead of IP addresses

### Backend Changes

1. **Removed ROS2 dependency** (`backend/main.py`)
   - Removed `rclpy` imports
   - Removed `ros_interface.ros_node` usage
   - Removed `/move`, `/stop`, `/ws/telemetry` endpoints (frontend uses ROS directly)
   - Kept `/health` and `/config` endpoints for system monitoring

2. **Updated requirements** (`requirements.txt`)
   - Removed rclpy note (ROS2 runs in Docker now)
   - Added `httpx` for health checks
   - Added optional `roslibpy` note if backend needs ROS access

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

2. **Add rosbridge to Docker container**:
   - Update your `bringup.launch.py` to include rosbridge_websocket and web_video_server
   - See `ros_launch/bringup_with_rosbridge.launch.py` for example

3. **Create frontend/.env file**:
   ```bash
   VITE_ROSBRIDGE_URL=ws://fordward.local:9090
   VITE_VIDEO_BASE=http://fordward.local:8080
   ```

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

- Frontend no longer connects to FastAPI WebSocket
- Backend `/move` and `/stop` endpoints removed (use ROS cmd_vel directly)
- Environment variables changed (VITE_API_BASE → VITE_ROSBRIDGE_URL, VITE_VIDEO_BASE)

## Migration Path

If you need to keep the old API endpoints for backward compatibility:

1. Install `roslibpy` in backend
2. Create a ROS client that connects to rosbridge
3. Publish to `/cmd_vel` via rosbridge instead of rclpy
4. Keep the `/move` and `/stop` endpoints as wrappers

See `docs/ARCHITECTURE.md` for more details.

