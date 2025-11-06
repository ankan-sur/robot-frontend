# Robot Connection Guide

This guide explains how to connect your interface (frontend) to the robot and how to set up the project on the robot itself.

## Overview

The project consists of:
- **Frontend**: React/TypeScript web interface (runs on your computer)
- **Backend**: FastAPI server (runs on the robot)
- **ROS2 Interface**: Bridges commands between backend and robot hardware

## Robot Connection Information

### WiFi Hotspot Mode (Default)

If the robot doesn't find its configured WiFi network on boot, it creates a hotspot:

- **SSID**: `Fordward`
- **Password**: `fordward`
- **Robot IP**: `192.168.149.1`
- **SSH User**: `pi`
- **SSH Password**: `fordward`

### Connecting to the Robot

#### Option 1: Via WiFi Hotspot (Recommended for first-time setup)

1. **Connect to Robot WiFi**:
   - On your computer, search for WiFi network `Fordward`
   - Connect using password `fordward`
   - Your computer will get an IP like `192.168.149.x`

2. **Verify Connection**:
   ```bash
   # Test SSH connection
   ssh pi@192.168.149.1
   # Password: fordward
   ```

3. **Copy Project to Robot**:
   - Your teammate will copy the entire project folder to a USB drive
   - Insert USB into robot
   - Copy project to robot (e.g., `~/robot_project/`)

#### Option 2: Via Configured WiFi

If the robot is configured to connect to a WiFi network (configured in `~/hiwonder_toolbox/wifi_conf.py`):
- The robot will have a different IP address on that network
- Find the robot's IP using: `ssh pi@<robot-ip>` or check your router's admin page

## Setting Up on Your Computer (Frontend)

### Step 1: Install Dependencies

```bash
cd frontend
npm install
```

### Step 2: Configure Environment

Create a `.env` file in the `frontend/` directory:

```bash
# For robot hotspot connection
VITE_API_BASE=http://192.168.149.1:8000

# Optional: If you set CONTROL_TOKEN on the backend
# VITE_CONTROL_TOKEN=your_token_here
```

**Note**: If the robot is on a different network, change `192.168.149.1` to the robot's actual IP address.

### Step 3: Run Frontend

```bash
cd frontend
npm run dev
```

The interface will open at `http://localhost:5173` and automatically connect to the robot backend.

## Setting Up on the Robot (Backend)

### Step 1: Transfer Project

1. Copy the entire project folder to the robot (via USB or SCP):
   ```bash
   # From your computer, if robot is on same network:
   scp -r /path/to/project pi@192.168.149.1:~/robot_project
   ```

2. Or copy from USB drive (if already on robot):
   ```bash
   # On robot via SSH
   cp -r /media/usb/project_folder ~/robot_project
   cd ~/robot_project
   ```

### Step 2: Install Python Dependencies

```bash
# On robot via SSH
cd ~/robot_project
pip install -r requirements.txt
```

**Note**: ROS2 (rclpy) should already be installed on the robot. If not, install ROS2 Humble following official documentation.

### Step 3: (Optional) Set Control Token

If you want to require authentication for robot control:

```bash
# On robot
export CONTROL_TOKEN=your_secret_token_here
# Or add to ~/.bashrc for persistence
```

### Step 4: Run Backend

```bash
# On robot via SSH
cd ~/robot_project
uvicorn backend.main:app --host 0.0.0.0 --port 8000
```

Or use the provided startup script:
```bash
chmod +x start_robot.sh
./start_robot.sh
```

The backend will be accessible at `http://192.168.149.1:8000` (or robot's IP).

### Step 5: Verify Backend is Running

Open a browser or use curl:
```bash
curl http://192.168.149.1:8000/status
```

You should see JSON telemetry data.

## Testing the Connection

1. **Start Backend on Robot**: Make sure the backend is running (see above)

2. **Start Frontend on Your Computer**: 
   ```bash
   cd frontend
   npm run dev
   ```

3. **Open Interface**: Navigate to `http://localhost:5173`

4. **Check Connection Status**: You should see "Connected" in green in the interface

5. **Test Robot Control**: Try the "Go to Lab" buttons or stop button

## Troubleshooting

### Frontend Can't Connect to Backend

1. **Check Robot IP**: Verify the robot's IP address:
   ```bash
   # On robot
   hostname -I
   ```

2. **Update .env File**: Make sure `VITE_API_BASE` in `frontend/.env` matches the robot's IP

3. **Check Backend is Running**: 
   ```bash
   curl http://192.168.149.1:8000/status
   ```

4. **Check Firewall**: The robot's firewall might block port 8000. Allow it:
   ```bash
   # On robot
   sudo ufw allow 8000
   ```

5. **Check CORS**: Make sure your frontend URL is allowed in `backend/main.py` CORS settings

### Backend Won't Start on Robot

1. **Check ROS2**: Verify ROS2 is installed:
   ```bash
   # On robot
   source /opt/ros/humble/setup.bash
   ros2 --help
   ```

2. **Check Python Dependencies**: 
   ```bash
   pip list | grep fastapi
   pip list | grep uvicorn
   ```

3. **Check Port Availability**: 
   ```bash
   # On robot
   sudo netstat -tulpn | grep 8000
   ```

### SSH Connection Issues

1. **Can't connect to hotspot**: 
   - Make sure you're connected to `Fordward` WiFi
   - Try disconnecting and reconnecting
   - Check if robot is powered on

2. **Wrong password**: Default password is `fordward` (all lowercase)

## Network Configuration Tips

### For Development (Local Network)

If both your computer and robot are on the same WiFi network:
1. Find robot's IP: `ssh pi@192.168.149.1` then `hostname -I`
2. Update `frontend/.env`: `VITE_API_BASE=http://<robot-ip>:8000`
3. Update `backend/main.py` CORS to include your computer's IP

### For Production

Consider:
- Setting up a static IP for the robot
- Configuring WiFi credentials in `~/hiwonder_toolbox/wifi_conf.py`
- Using a reverse proxy (nginx) for better security
- Setting up SSL certificates for HTTPS

## Quick Reference

| Item | Value |
|------|-------|
| Robot Hotspot SSID | `Fordward` |
| Robot Hotspot Password | `fordward` |
| Robot IP (Hotspot) | `192.168.149.1` |
| SSH User | `pi` |
| SSH Password | `fordward` |
| Backend Port | `8000` |
| Frontend Port | `5173` |
| Backend URL | `http://192.168.149.1:8000` |
| Frontend URL | `http://localhost:5173` |

## For Your Teammate (Ricky)

When receiving the robot:

1. **Power on the robot** and wait for it to boot
2. **Connect to WiFi hotspot** `Fordward` (password: `fordward`)
3. **SSH into robot**: `ssh pi@192.168.149.1` (password: `fordward`)
4. **Copy project from USB**:
   ```bash
   # Find USB drive
   ls /media/pi/
   # Copy project
   cp -r /media/pi/USB_NAME/robot_project ~/robot_project
   cd ~/robot_project
   ```
5. **Install dependencies** (if needed): `pip install -r requirements.txt`
6. **Run backend**: `uvicorn backend.main:app --host 0.0.0.0 --port 8000`
7. **On your computer**, configure `frontend/.env` with `VITE_API_BASE=http://192.168.149.1:8000`
8. **Run frontend**: `cd frontend && npm run dev`

## Additional Resources

- FastAPI docs (when backend is running): `http://192.168.149.1:8000/docs`
- ROS2 topic monitoring: `ros2 topic echo /hiwonder_cmd` (on robot)
- Check backend logs for errors

