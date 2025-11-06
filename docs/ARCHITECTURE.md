# HFH Robot Architecture (Refactored)

## Overview

This architecture separates ROS2 from the backend API, using rosbridge_websocket as the bridge between the web frontend and ROS2 topics running in a Docker container.

```
┌─────────────────────────────────────────┐
│  Docker Container (--net=host)           │
│  ├─ ROS2 (bringup.launch.py)            │
│  │  ├─ /cmd_vel                          │
│  │  ├─ /odom                             │
│  │  ├─ /ros_robot_controller/battery     │
│  │  └─ /ascamera/camera_publisher/rgb0/image │
│  ├─ rosbridge_websocket (port 9090)     │
│  └─ web_video_server (port 8080)        │
└─────────────────────────────────────────┘
         ↑                    ↑
         │                    │
    Frontend              Backend (optional)
  (roslib.js)           (roslibpy if needed)
```

## Key Components

### Frontend (`frontend/`)
- **Technology**: React + TypeScript + Vite
- **ROS Connection**: Uses `roslib` to connect to `rosbridge_websocket` at `ws://fordward.local:9090`
- **Topics Used**:
  - `/cmd_vel` - Send movement commands (geometry_msgs/Twist)
  - `/odom` - Receive position/odometry (nav_msgs/Odometry)
  - `/ros_robot_controller/battery` - Battery status
  - `/ascamera/camera_publisher/rgb0/image` - Camera feed (via web_video_server)

### Backend (`backend/`)
- **Technology**: FastAPI
- **Purpose**: Optional REST API for configuration and health checks
- **ROS Access**: None (ROS2 runs in Docker, frontend connects directly via rosbridge)
- **Note**: If backend needs ROS access, use `roslibpy` (NOT `rclpy`)

### ROS2 Docker Container
- **Network**: `--net=host` (allows access to host network)
- **Launch**: `ros2 launch bringup bringup.launch.py`
- **Additional Services**:
  - `rosbridge_websocket` on port 9090
  - `web_video_server` on port 8080

## Setup Instructions

### 1. Install Frontend Dependencies

```bash
cd frontend
npm install
```

### 2. Configure Environment

Create `frontend/.env`:

```bash
VITE_ROSBRIDGE_URL=ws://fordward.local:9090
VITE_VIDEO_BASE=http://fordward.local:8080
```

### 3. Add rosbridge to Docker Container

Update your `bringup.launch.py` or create a new launch file:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Your existing bringup nodes...
        
        # Add rosbridge_websocket
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            parameters=[{
                'port': 9090,
                'address': '0.0.0.0',
            }],
            output='screen'
        ),
        
        # Add web_video_server
        Node(
            package='web_video_server',
            executable='web_video_server',
            name='web_video_server',
            parameters=[{
                'port': 8080,
            }],
            output='screen'
        ),
    ])
```

### 4. Build Frontend

```bash
cd frontend
npm run build
```

### 5. Run Frontend (Development)

```bash
cd frontend
npm run dev
```

## Topics Reference

Check message types:

```bash
# On robot (in Docker or host)
ros2 topic info /ros_robot_controller/battery
ros2 topic info /odom
ros2 topic info /cmd_vel
ros2 topic info /ascamera/camera_publisher/rgb0/image
```

Update `frontend/src/ros/config.ts` if message types differ.

## Troubleshooting

### Frontend Can't Connect to rosbridge

1. Check rosbridge is running:
   ```bash
   ros2 topic list  # Should show topics
   ```

2. Check port 9090 is accessible:
   ```bash
   curl http://fordward.local:9090  # Should fail (not HTTP), but port should be open
   ```

3. Check firewall:
   ```bash
   sudo ufw allow 9090
   ```

### Video Not Showing

1. Check web_video_server is running:
   ```bash
   curl http://fordward.local:8080/stream_viewer
   ```

2. Check camera topic is publishing:
   ```bash
   ros2 topic echo /ascamera/camera_publisher/rgb0/image --once
   ```

3. Verify video URL in browser:
   ```
   http://fordward.local:8080/stream?topic=/ascamera/camera_publisher/rgb0/image&type=webp
   ```

### Battery Topic Message Type

If battery topic uses a different message type, update:

1. Check actual type:
   ```bash
   ros2 topic info /ros_robot_controller/battery
   ```

2. Update `frontend/src/ros/config.ts`:
   ```typescript
   messageTypes: {
     battery: 'std_msgs/UInt8',  // or whatever type it is
   }
   ```

3. Update `frontend/src/ros/hooks.ts` to extract the correct field from the message.

## Migration Notes

### What Changed

1. **Removed**: `rclpy` dependency from backend
2. **Removed**: `ros_interface/ros_node.py` usage
3. **Added**: `roslib` for frontend ROS connection
4. **Added**: `rosbridge_websocket` in Docker container
5. **Added**: `web_video_server` for camera streaming
6. **Changed**: Frontend now connects directly to ROS via rosbridge
7. **Changed**: All references from IP addresses to `fordward.local` hostname

### Backward Compatibility

The old FastAPI endpoints (`/move`, `/stop`, `/status`) are removed. If you need them:
- Use `roslibpy` in backend to publish to `/cmd_vel` via rosbridge
- Or keep the old endpoints as wrappers around roslibpy

## Next Steps

1. Test rosbridge connection from frontend
2. Verify battery topic message type and update if needed
3. Implement proper navigation (Nav2 action client) instead of simple cmd_vel
4. Add error handling and reconnection logic
5. Add map visualization using /map topic (if available)

