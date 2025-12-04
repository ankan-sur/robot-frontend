# ROS Integration

Complete reference for ROS topics, services, and actions used by the frontend.

## Topics (Subscribed)

### /amcl_pose or /robot/pose
Robot position in map frame.

| Property | Value |
|----------|-------|
| Type | `geometry_msgs/PoseWithCovarianceStamped` or `PoseStamped` |
| Rate | ~10 Hz (throttled in UI) |
| Used by | `useRobotPose()` hook, `MapView` component |

### /ros_robot_controller/battery
Battery voltage in millivolts.

| Property | Value |
|----------|-------|
| Type | `std_msgs/UInt16` |
| Rate | ~0.5 Hz |
| Range | 6000-8400 mV (2S LiPo) |
| Used by | `useBattery()` hook |

The hook converts millivolts to percentage:
- 6000 mV = 0%
- 8400 mV = 100%

### /map
Occupancy grid from SLAM or map_server.

| Property | Value |
|----------|-------|
| Type | `nav_msgs/OccupancyGrid` |
| QoS | Transient local |
| Used by | `useMap()` hook, `MapView` component |

Grid values:
- `-1` = Unknown (gray)
- `0` = Free (white)
- `100` = Occupied (black)

### /rosout
ROS console output.

| Property | Value |
|----------|-------|
| Type | `rcl_interfaces/Log` |
| Used by | `DebugLog` component |

## Topics (Published)

### /ui/cmd_vel
Teleop velocity commands.

| Property | Value |
|----------|-------|
| Type | `geometry_msgs/Twist` |
| Published by | `useCmdVel()` hook |
| Fallback | `/cmd_vel` if `/ui/cmd_vel` unavailable |

Message structure:
```json
{
  "linear": { "x": 0.5, "y": 0, "z": 0 },
  "angular": { "x": 0, "y": 0, "z": 0.3 }
}
```

## Services

### /get_mode
Get current operating mode and active map.

| Property | Value |
|----------|-------|
| Type | `std_srvs/Trigger` |
| Response | JSON in `message` field |

Response format:
```json
{
  "mode": "localization",
  "map": "office_map"
}
```

### /set_mode
Switch operating mode.

| Property | Value |
|----------|-------|
| Type | `interfaces/SetString` |
| Request | `{ data: "slam" }` or `"localization"` or `"idle"` |

### /stop_slam_and_save
Save current map and stop SLAM.

| Property | Value |
|----------|-------|
| Type | `interfaces/SetString` |
| Request | `{ data: "map_name" }` |
| Saves to | `~/ros2_ws/src/robothome/src/slam/maps/` |

### /load_map
Load a saved map for localization.

| Property | Value |
|----------|-------|
| Type | `interfaces/SetString` |
| Request | `{ data: "map_name" }` |

### /list_maps
Get list of available maps.

| Property | Value |
|----------|-------|
| Type | `std_srvs/Trigger` |
| Response | JSON array in `message` field |

Response format:
```json
["office_map", "warehouse", "test_area"]
```

### /navigate_to_pose/cancel
Cancel active navigation goals.

| Property | Value |
|----------|-------|
| Type | `action_msgs/CancelGoal` |
| Request | `{}` (cancels all) |

## Actions

### /navigate_to_pose
Send navigation goal (Nav2).

| Property | Value |
|----------|-------|
| Type | `nav2_msgs/action/NavigateToPose` |
| Used by | `useNavigateToPose()` hook |

Goal message:
```json
{
  "pose": {
    "header": { "frame_id": "map" },
    "pose": {
      "position": { "x": 1.0, "y": 2.0, "z": 0 },
      "orientation": { "x": 0, "y": 0, "z": 0, "w": 1 }
    }
  }
}
```

## Configuration

All topic/service names are defined in `frontend/src/ros/config.ts`:

```typescript
export const ROS_CONFIG = {
  rosbridgeUrl: 'ws://<host>:9090',
  videoBase: 'http://<host>:8080',
  
  topics: {
    cmdVel: '/ui/cmd_vel',
    odom: '/odom',
    battery: '/ros_robot_controller/battery',
    camera: '/ascamera/camera_publisher/rgb0/image',
    map: '/map',
    robotState: '/robot/state',
    rosout: '/rosout',
  },
  
  services: {
    setMode: '/set_mode',
    getMode: '/get_mode',
    stopSlamAndSave: '/stop_slam_and_save',
    loadMap: '/load_map',
    listMaps: '/list_maps',
    navCancel: '/navigate_to_pose/cancel',
  },
  
  actions: {
    navigateToPose: '/navigate_to_pose',
  }
}
```

## Video Stream

Camera feed uses web_video_server (not rosbridge).

| Property | Value |
|----------|-------|
| URL | `http://<host>:8080/stream?topic=/ascamera/camera_publisher/rgb0/image&type=mjpeg` |
| Port | 8080 |
| Format | MJPEG over HTTP |

## Troubleshooting

### Topics not receiving data

1. Check rosbridge is running: `nc -zv <host> 9090`
2. Verify topic exists: `ros2 topic list`
3. Check topic has data: `ros2 topic echo <topic> --once`

### Services failing

1. Check service exists: `ros2 service list`
2. Test service: `ros2 service call /get_mode std_srvs/srv/Trigger`

### Video not loading

1. Check web_video_server: `curl -I http://<host>:8080`
2. Test stream URL directly in browser
3. Verify camera topic: `ros2 topic hz /ascamera/camera_publisher/rgb0/image`
