# Raspberry Pi Setup

Setting up a fresh Raspberry Pi for the robot.

## OS Installation

1. Flash Raspberry Pi OS (64-bit) to SD card
2. Enable SSH during setup
3. Set hostname to `fordward`

## Initial Configuration

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y build-essential git curl wget vim
```

## Set Hostname

```bash
sudo hostnamectl set-hostname fordward
echo "127.0.1.1 fordward" | sudo tee -a /etc/hosts
sudo reboot
```

## Enable mDNS

```bash
sudo apt install -y avahi-daemon avahi-utils
sudo systemctl enable --now avahi-daemon
```

Test from another machine:

```bash
ping fordward.local
```

## Install ROS 2 Humble

```bash
# Locale setup
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS 2 repo
sudo apt install -y software-properties-common curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2-latest.list

# Install ROS 2
sudo apt update
sudo apt install -y ros-humble-desktop python3-rosdep python3-colcon-common-extensions
sudo rosdep init || true
rosdep update

# Add to bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Install Bridge Packages

```bash
sudo apt install -y \
  ros-humble-rosbridge-server \
  ros-humble-web-video-server \
  ros-humble-rosboard
```

## Configure Firewall

```bash
sudo ufw allow 22    # SSH
sudo ufw allow 9090  # rosbridge
sudo ufw allow 8080  # web_video_server
sudo ufw allow 8888  # rosboard
sudo ufw enable
```

## Node.js (Optional)

For building frontend on the Pi:

```bash
curl -fsSL https://deb.nodesource.com/setup_20.x | sudo -E bash -
sudo apt install -y nodejs
```

## Verification

```bash
# ROS 2
source /opt/ros/humble/setup.bash
ros2 topic list

# Rosbridge (after launching)
nc -zv localhost 9090

# Web video server (after launching)
curl -I http://localhost:8080
```

## Next Steps

1. Clone your robot workspace
2. Build with colcon
3. Create launch files including rosbridge and web_video_server
4. Set up systemd service for auto-start
