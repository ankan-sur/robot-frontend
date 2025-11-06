# Quick Setup Guide for Robot (Ricky)

This is a quick reference for setting up the project on the robot.

## Initial Setup Steps

### 1. Connect to Robot

1. Power on the robot
2. Connect to WiFi hotspot: `Fordward` (password: `fordward`)
3. SSH into robot:
   ```bash
   ssh pi@192.168.149.1
   # Password: fordward
   ```

### 2. Transfer Project

**Option A: From USB Drive**
```bash
# On robot via SSH
# Find your USB drive
ls /media/pi/

# Copy project (replace USB_NAME with your USB name)
cp -r /media/pi/USB_NAME/ece480_capstone_henry_ford_health ~/robot_project
cd ~/robot_project
```

**Option B: From Network (if on same WiFi)**
```bash
# From your computer
scp -r /path/to/project pi@192.168.149.1:~/robot_project
```

### 3. Install Dependencies

```bash
# On robot
cd ~/robot_project

# Install Python dependencies
pip install -r requirements.txt

# Or use the startup script (it will install for you)
chmod +x start_robot.sh
```

### 4. Run Backend

```bash
# Simple way
uvicorn backend.main:app --host 0.0.0.0 --port 8000

# Or use the startup script
./start_robot.sh
```

### 5. Verify It's Running

Open in browser or test with curl:
```bash
curl http://192.168.149.1:8000/status
```

You should see JSON response with robot status.

## Running on Startup (Optional)

To make the backend start automatically when robot boots:

```bash
# Create systemd service
sudo nano /etc/systemd/system/robot-backend.service
```

Add this content:
```ini
[Unit]
Description=HFH Robot Backend
After=network.target

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi/robot_project
Environment="PATH=/home/pi/robot_project/venv/bin:/usr/local/bin:/usr/bin:/bin"
ExecStart=/home/pi/robot_project/venv/bin/uvicorn backend.main:app --host 0.0.0.0 --port 8000
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
```

Then enable it:
```bash
sudo systemctl enable robot-backend.service
sudo systemctl start robot-backend.service
```

## Troubleshooting

- **Can't SSH**: Make sure you're connected to `Fordward` WiFi
- **Port already in use**: Kill existing process: `sudo killall uvicorn`
- **Import errors**: Make sure ROS2 is sourced: `source /opt/ros/humble/setup.bash`
- **Permission denied**: Use `chmod +x start_robot.sh`

## Quick Commands

```bash
# Check if backend is running
curl http://192.168.149.1:8000/status

# Check backend logs (if using systemd)
sudo journalctl -u robot-backend.service -f

# Stop backend
sudo systemctl stop robot-backend.service

# Restart backend
sudo systemctl restart robot-backend.service
```

