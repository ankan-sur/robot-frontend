# Connection Guide

How to connect the web UI to the robot.

## Network Setup

**Robot WiFi Hotspot:**

- SSID: `Fordward`
- Password: `fordward`
- Robot IP: `192.168.149.1`

**SSH Access:**

```bash
ssh pi@192.168.149.1
# Password: fordward
```

## Starting the UI

### Development (on your computer)

```bash
cd frontend
npm install
npm run dev -- --host
```

Open `http://localhost:5173` or `http://fordward.local:5173`

### Production (on robot)

See [DEPLOYMENT.md](./DEPLOYMENT.md) for Nginx setup.

## Verifying Connection

### 1. Check rosbridge

```bash
nc -zv fordward.local 9090
# Should show: Connection succeeded
```

### 2. Check video server

```bash
curl -I http://fordward.local:8080
# Should return HTTP 200
```

### 3. Check ROS topics

```bash
ssh pi@fordward.local
ros2 topic list
```

Expected topics:

- `/map`
- `/odom`
- `/ros_robot_controller/battery`
- `/rosout`

### 4. Test camera stream

Open in browser:

```
http://fordward.local:8080/stream?topic=/ascamera/camera_publisher/rgb0/image&type=mjpeg
```

## Troubleshooting

### "Disconnected" in UI

1. Verify robot is powered on
2. Check you're on the robot's WiFi network
3. Confirm rosbridge is running: `ros2 node list | grep rosbridge`
4. Check firewall: `sudo ufw allow 9090`

### Map not displaying

1. Check mode is SLAM or Nav (not Idle)
2. Verify `/map` topic: `ros2 topic echo /map --once`
3. In SLAM mode, drive around to build map

### Camera not loading

1. Check web_video_server: `ros2 node list | grep web_video`
2. Verify camera topic: `ros2 topic hz /ascamera/camera_publisher/rgb0/image`
3. Test stream URL directly in browser

### Teleop not working

1. Check `/ui/cmd_vel` or `/cmd_vel` topic exists
2. Verify teleop_gateway is running
3. Check keyboard focus is on the teleop area

## Ports Reference

| Port | Service |
|------|---------|
| 9090 | rosbridge_websocket |
| 8080 | web_video_server |
| 8888 | rosboard (optional) |
| 5173 | Vite dev server |
| 22 | SSH |
