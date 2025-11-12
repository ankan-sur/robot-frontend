# Serve the UI at fordward.local

This makes the frontend reachable for everyone on the network at:

- http://fordward.local

Follow these steps on the Raspberry Pi (robot):

## 1) Set the hostname to `fordward`

```bash
sudo hostnamectl set-hostname fordward
echo "127.0.1.1 fordward" | sudo tee -a /etc/hosts
```

Reboot (or restart networking) to apply:

```bash
sudo reboot
```

## 2) Enable mDNS (Avahi)

Install and start Avahi so `fordward.local` resolves on the LAN:

```bash
sudo apt update
sudo apt install -y avahi-daemon avahi-utils
sudo systemctl enable --now avahi-daemon
```

Test from another device on the same network:

```bash
ping fordward.local
```

Note: Windows may require Apple Bonjour for `.local` discovery.

## 3) Serve the frontend

Option A: Development (Vite dev server, port 5173)

```bash
cd frontend
npm install
npm run dev -- --host
# then open http://fordward.local:5173
```

Option B: Production build + preview

```bash
cd frontend
npm install
npm run build
npm run preview -- --host
# opens on port 4173 by default -> http://fordward.local:4173
```

Option C: Nginx on port 80 (recommended for kiosks)

```bash
cd frontend
npm ci && npm run build
sudo apt install -y nginx
sudo tee /etc/nginx/sites-available/robot-ui >/dev/null <<'NGINX'
server {
  listen 80 default_server;
  listen [::]:80 default_server;
  server_name fordward.local;

  root /home/pi/robot_project/frontend/dist;
  index index.html;

  location / {
    try_files $uri $uri/ /index.html;
  }
}
NGINX
sudo ln -sf /etc/nginx/sites-available/robot-ui /etc/nginx/sites-enabled/robot-ui
sudo nginx -t && sudo systemctl restart nginx
# open http://fordward.local
```

## 4) Frontend config

The app auto-targets services on the current host (`window.location.hostname`).
If you serve the UI at `http://fordward.local`, it will connect to:

- rosbridge: `ws://fordward.local:9090`
- video: `http://fordward.local:8080`

No `.env` changes are required. If ports differ, set env overrides in `frontend/.env`.

## 5) Useful URLs

- Rosboard: `http://fordward.local:8888`
- Camera (viewer): `http://fordward.local:8080/stream_viewer?topic=/ascamera/camera_publisher/rgb0/image`
- Camera (mjpeg): `http://fordward.local:8080/stream?topic=/ascamera/camera_publisher/rgb0/image&type=mjpeg`
