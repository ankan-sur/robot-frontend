# Fresh Raspberry Pi Setup Guide

Set up a fresh Raspberry Pi to run the robot stack expected by this UI: ROS 2 (Humble) + rosbridge_websocket + web_video_server + rosboard + mDNS. No Python backend is required for normal operation.

Prerequisites
- Raspberry Pi (64‑bit OS recommended)
- Internet connectivity (Wi‑Fi or Ethernet)
- SSH access or console

1) Base OS setup
- Flash Raspberry Pi OS and enable SSH.
- Update system:
  - `sudo apt update && sudo apt upgrade -y`
  - `sudo apt install -y build-essential git curl wget vim`

2) Node.js (for building UI on the Pi, optional)
- Install LTS:
  - `curl -fsSL https://deb.nodesource.com/setup_20.x | sudo -E bash -`
  - `sudo apt install -y nodejs`
- Verify: `node -v && npm -v`

3) Python (optional)
- Only needed for ROS tools or scripts (no backend needed):
  - `sudo apt install -y python3 python3-pip python3-venv`
  - `pip3 install --upgrade pip`

4) ROS 2 Humble
- Locale and repo prep (Ubuntu Jammy base instructions):
  - `sudo apt install -y locales software-properties-common`
  - `sudo locale-gen en_US en_US.UTF-8`
  - `sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8`
  - `export LANG=en_US.UTF-8`
  - `sudo add-apt-repository universe`
  - `sudo apt update && sudo apt install -y curl gnupg lsb-release`
  - `sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -`
  - `echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2-latest.list`
- Install ROS 2:
  - `sudo apt update`
  - `sudo apt install -y ros-humble-desktop python3-rosdep python3-colcon-common-extensions`
  - `sudo rosdep init || true && rosdep update`
  - `echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc`
  - `source ~/.bashrc`

5) Install bridge + tools
- `sudo apt install -y ros-humble-rosbridge-server ros-humble-web-video-server ros-humble-rosboard`
- mDNS for `fordward.local`:
  - `sudo apt install -y avahi-daemon avahi-utils`
  - `sudo systemctl enable --now avahi-daemon`

6) Hostname (recommended)
- `sudo hostnamectl set-hostname fordward`
- `echo "127.0.1.1 fordward" | sudo tee -a /etc/hosts`
- Reboot: `sudo reboot`

7) Bringup
- Ensure your launch includes:
  - rosbridge_server/rosbridge_websocket (port 9090, address 0.0.0.0)
  - web_video_server (port 8080)
  - rosboard (port 8888)
  - system_topics node (publishes `/map`, `/pois`, `/robot/state`, `/available_maps`, `/connected_clients`, `/client_count`, `/navigate_to_pose/status`; provides `/navigate_to_pose/cancel`)

8) Frontend (run on your computer or on the robot)
- On your dev machine:
  - `cd frontend && npm install && npm run dev -- --host`
  - Open `http://fordward.local:5173` (or use the robot IP)
- For production serving via Nginx, see `docs/HOSTNAME_AND_SERVE.md`.

9) Verification
- ROS 2: `source /opt/ros/humble/setup.bash && ros2 topic list`
- rosbridge: `nc -zv fordward.local 9090`
- WVS: `curl -I http://fordward.local:8080/stream_viewer | head -n1`
- Rosboard: `curl -I http://fordward.local:8888 | head -n1`
- UI: browse to `http://fordward.local:5173` (dev) or `http://fordward.local` (nginx)

10) Firewall (if UFW is enabled)
- `sudo ufw allow 9090`  # rosbridge
- `sudo ufw allow 8080`  # web_video_server
- `sudo ufw allow 8888`  # rosboard

Notes
- Use 64‑bit OS for better compatibility and performance.
- SSD strongly recommended over SD cards for reliability.
