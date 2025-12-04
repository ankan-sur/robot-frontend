# Robot Setup Checklist

Quick reference for getting the robot stack ready for the UI.

## Power On

1. Power on the robot
2. Wait for boot (~30 seconds)
3. Connect to WiFi `Fordward` (password: `fordward`)

## Verify ROS Stack

SSH into robot:

```bash
ssh pi@192.168.149.1
```

Check nodes are running:

```bash
ros2 node list
```

Expected nodes:

- `/rosbridge_websocket`
- `/web_video_server`
- `/mode_manager`
- `/teleop_gateway`
- `/ros_robot_controller`

## Check Required Topics

```bash
ros2 topic list | grep -E "map|odom|battery|cmd_vel"
```

Expected:

- `/map` - Occupancy grid
- `/odom` - Odometry
- `/ros_robot_controller/battery` - Battery voltage
- `/ui/cmd_vel` - Teleop commands

## Check Services

```bash
ros2 service list | grep -E "mode|map"
```

Expected:

- `/get_mode`
- `/set_mode`
- `/load_map`
- `/list_maps`
- `/stop_slam_and_save`

## Test Endpoints

From another machine on the network:

```bash
# Rosbridge
nc -zv fordward.local 9090

# Video server
curl -I http://fordward.local:8080

# Camera stream
curl -I "http://fordward.local:8080/stream?topic=/ascamera/camera_publisher/rgb0/image"
```

## Firewall (if enabled)

```bash
sudo ufw allow 9090  # rosbridge
sudo ufw allow 8080  # video server
sudo ufw allow 8888  # rosboard (optional)
```

## Start UI

On your development machine:

```bash
cd frontend
npm install
npm run dev -- --host
```

Open `http://fordward.local:5173`

## Common Issues

### Nodes not starting

Check launch logs:

```bash
ros2 launch bringup core_bringup.launch.py
```

### Battery topic missing

```bash
ros2 topic info /ros_robot_controller/battery
```

### Camera not streaming

```bash
ros2 topic hz /ascamera/camera_publisher/rgb0/image
```
