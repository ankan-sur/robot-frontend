# Deployment

Serving the frontend in production.

## Build the Frontend

```bash
cd frontend
npm ci
npm run build
```

This creates `frontend/dist/` with static files.

## Option 1: Vite Preview

Quick way to serve the production build:

```bash
npm run preview -- --host --port 4173
```

Access at `http://fordward.local:4173`

## Option 2: Nginx (Recommended)

### Install Nginx

```bash
sudo apt install -y nginx
```

### Configure Site

```bash
sudo tee /etc/nginx/sites-available/robot-ui << 'EOF'
server {
    listen 80 default_server;
    listen [::]:80 default_server;
    server_name fordward.local;

    root /home/pi/robot-frontend/frontend/dist;
    index index.html;

    location / {
        try_files $uri $uri/ /index.html;
    }
}
EOF
```

### Enable Site

```bash
sudo ln -sf /etc/nginx/sites-available/robot-ui /etc/nginx/sites-enabled/
sudo rm -f /etc/nginx/sites-enabled/default
sudo nginx -t
sudo systemctl restart nginx
```

Access at `http://fordward.local`

## Auto-Start on Boot

### Systemd Service for ROS

Create `/etc/systemd/system/ros-bringup.service`:

```ini
[Unit]
Description=ROS 2 Robot Bringup
After=network.target

[Service]
Type=simple
User=pi
Environment="ROS_DOMAIN_ID=0"
ExecStart=/bin/bash -c "source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash && ros2 launch bringup core_bringup.launch.py"
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
```

Enable:

```bash
sudo systemctl daemon-reload
sudo systemctl enable ros-bringup
sudo systemctl start ros-bringup
```

## Environment Configuration

The frontend auto-detects URLs from the browser hostname. No `.env` file needed for standard setup.

To override (create `frontend/.env`):

```env
VITE_ROSBRIDGE_URL=ws://192.168.1.100:9090
VITE_VIDEO_BASE=http://192.168.1.100:8080
```

## Common URLs

| URL | Description |
|-----|-------------|
| `http://fordward.local` | Production UI (Nginx) |
| `http://fordward.local:5173` | Development UI |
| `http://fordward.local:4173` | Preview build |
| `http://fordward.local:8080` | Video server |
| `http://fordward.local:8888` | Rosboard |
