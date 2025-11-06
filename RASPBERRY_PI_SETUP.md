# Fresh Raspberry Pi Setup Guide

This guide will help you set up a completely fresh Raspberry Pi with all the necessary software for this project.

## Prerequisites

- Raspberry Pi (with SD card)
- Raspberry Pi OS installed (preferably Raspberry Pi OS 64-bit)
- Internet connection (via WiFi or Ethernet)
- SSH access or direct access to the Pi

## Step 1: Initial Raspberry Pi OS Setup

1. **Flash Raspberry Pi OS** to SD card using [Raspberry Pi Imager](https://www.raspberrypi.com/software/)
2. **Enable SSH** (if not already enabled):
   - Create empty file named `ssh` in the boot partition
   - Or enable via `raspi-config` after first boot
3. **Boot the Pi** and connect via SSH or directly

## Step 2: Update System Packages

```bash
# Update package lists
sudo apt update

# Upgrade all packages
sudo apt upgrade -y

# Install essential build tools
sudo apt install -y build-essential git curl wget vim
```

## Step 3: Install Python and pip

```bash
# Install Python 3 and pip (usually pre-installed, but ensure latest)
sudo apt install -y python3 python3-pip python3-venv

# Verify installation
python3 --version
pip3 --version

# Upgrade pip to latest version
pip3 install --upgrade pip
```

## Step 4: Install Node.js and npm

### Option A: Using NodeSource Repository (Recommended - Latest LTS)

```bash
# Install Node.js 20.x LTS (or latest LTS)
curl -fsSL https://deb.nodesource.com/setup_20.x | sudo -E bash -
sudo apt install -y nodejs

# Verify installation
node --version
npm --version
```

### Option B: Using apt (Easier, but older version)

```bash
# Install Node.js and npm from Raspberry Pi repositories
sudo apt install -y nodejs npm

# Verify installation
node --version
npm --version

# If version is too old, use Option A instead
```

### Option C: Using nvm (Node Version Manager - Most Flexible)

```bash
# Install nvm
curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.0/install.sh | bash

# Reload shell configuration
source ~/.bashrc

# Install latest LTS Node.js
nvm install --lts
nvm use --lts

# Verify installation
node --version
npm --version
```

## Step 5: Install ROS2 Humble

ROS2 Humble is required for the robot interface. This is a critical step.

### Install ROS2 Humble on Ubuntu 22.04 (Raspberry Pi OS Bullseye/Bookworm)

```bash
# Set locale
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS2 apt repository
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl gnupg lsb-release

# Add ROS2 GPG key
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

# Install ROS2 Humble
sudo apt update
sudo apt install -y ros-humble-desktop

# Install additional ROS2 tools
sudo apt install -y python3-rosdep python3-colcon-common-extensions

# Initialize rosdep
sudo rosdep init
rosdep update

# Source ROS2 setup
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

**Note**: If you're using Raspberry Pi OS (Debian-based), you may need to adjust the repository URL. For Raspberry Pi OS Bookworm, you might need to use Ubuntu Jammy repositories.

### Alternative: Install ROS2 from Source (if apt doesn't work)

If the above doesn't work for your Raspberry Pi OS version, you may need to build ROS2 from source. This is more complex but guarantees compatibility.

```bash
# Install dependencies
sudo apt install -y \
  build-essential \
  cmake \
  git \
  libbullet-dev \
  python3-colcon-common-extensions \
  python3-flake8 \
  python3-pip \
  python3-pytest-cov \
  python3-rosdep \
  python3-setuptools \
  python3-vcstool \
  wget

# Install more dependencies
python3 -m pip install -U \
  argcomplete \
  flake8-blind-except \
  flake8-builtins \
  flake8-class-newline \
  flake8-comprehensions \
  flake8-deprecated \
  flake8-docstrings \
  flake8-import-order \
  flake8-quotes \
  pytest-repeat \
  pytest-rerunfailures \
  pytest \
  pytest-cov \
  pytest-runner \
  setuptools

# Create workspace and clone ROS2
mkdir -p ~/ros2_humble/src
cd ~/ros2_humble
wget https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos
vcs import src < ros2.repos

# Build ROS2
cd ~/ros2_humble
colcon build --symlink-install

# Source ROS2
echo "source ~/ros2_humble/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Step 6: Install Additional System Dependencies

```bash
# Install additional packages that might be needed
sudo apt install -y \
  python3-dev \
  libffi-dev \
  libssl-dev \
  python3-setuptools \
  python3-wheel
```

## Step 7: Set Up Project

```bash
# Navigate to your project directory
cd ~

# If you have the project on USB, copy it
# cp -r /media/pi/USB_NAME/ece480_capstone_henry_ford_health ~/robot_project
# cd ~/robot_project

# Or clone from git (if using git)
# git clone <your-repo-url> robot_project
# cd ~/robot_project
```

## Step 8: Install Python Dependencies

```bash
# Navigate to project directory
cd ~/robot_project

# Create virtual environment (recommended)
python3 -m venv venv
source venv/bin/activate

# Install Python packages
pip install -r requirements.txt

# Or install manually
pip install fastapi==0.115.5 uvicorn[standard]==0.30.6 pydantic>=2.7,<3
```

**Important**: `rclpy` (ROS2 Python client library) should be installed as part of ROS2, not via pip. Verify it's available:

```bash
# Test ROS2 installation
source /opt/ros/humble/setup.bash  # or source ~/ros2_humble/install/setup.bash if built from source
python3 -c "import rclpy; print('ROS2 Python library is working!')"
```

## Step 9: Install Frontend Dependencies

```bash
# Navigate to frontend directory
cd ~/robot_project/frontend

# Install npm packages
npm install

# Verify installation
npm --version
```

## Step 10: Verify Everything Works

### Test ROS2:
```bash
source /opt/ros/humble/setup.bash
ros2 --help
```

### Test Python:
```bash
python3 --version
pip3 list | grep fastapi
```

### Test Node.js:
```bash
node --version
npm --version
```

### Test Project Setup:
```bash
cd ~/robot_project

# Activate virtual environment
source venv/bin/activate

# Source ROS2 (if not in .bashrc yet)
source /opt/ros/humble/setup.bash

# Try importing Python modules
python3 -c "from backend.main import app; print('Backend imports OK')"
```

## Step 11: Configure WiFi (Optional)

If you want the robot to connect to a WiFi network automatically:

```bash
# Edit WiFi configuration
nano ~/hiwonder_toolbox/wifi_conf.py

# Add your WiFi credentials
```

If WiFi is not configured, the robot will create the `Fordward` hotspot.

## Step 12: Make Scripts Executable

```bash
cd ~/robot_project
chmod +x start_robot.sh
chmod +x test_connection.sh
```

## Step 13: Test the Backend

```bash
cd ~/robot_project
source venv/bin/activate
source /opt/ros/humble/setup.bash

# Start the backend
uvicorn backend.main:app --host 0.0.0.0 --port 8000
```

In another terminal or browser, test:
```bash
curl http://localhost:8000/status
```

## Quick Install Script

For convenience, here's a combined script you can run:

```bash
#!/bin/bash
# Save as setup_pi.sh and run: chmod +x setup_pi.sh && ./setup_pi.sh

set -e

echo "=== Updating system ==="
sudo apt update && sudo apt upgrade -y

echo "=== Installing essential tools ==="
sudo apt install -y build-essential git curl wget vim python3 python3-pip python3-venv

echo "=== Installing Node.js ==="
curl -fsSL https://deb.nodesource.com/setup_20.x | sudo -E bash -
sudo apt install -y nodejs

echo "=== Installing Python packages ==="
pip3 install --upgrade pip

echo "=== Installing ROS2 dependencies ==="
sudo apt install -y locales software-properties-common
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

echo "=== Node.js version: $(node --version) ==="
echo "=== npm version: $(npm --version) ==="
echo "=== Python version: $(python3 --version) ==="
echo "=== pip version: $(pip3 --version) ==="

echo ""
echo "=== Setup complete! ==="
echo "Next steps:"
echo "1. Install ROS2 Humble (see Step 5 above)"
echo "2. Copy your project to ~/robot_project"
echo "3. Install project dependencies: cd ~/robot_project && pip install -r requirements.txt"
echo "4. Install frontend dependencies: cd ~/robot_project/frontend && npm install"
```

## Troubleshooting

### ROS2 Installation Issues

**Problem**: ROS2 repository not found for your OS version
**Solution**: Check your Raspberry Pi OS version:
```bash
lsb_release -a
```
If it's not Ubuntu 22.04 compatible, you may need to build ROS2 from source (see Step 5 Alternative).

### Node.js Version Too Old

**Problem**: `node --version` shows v12 or older
**Solution**: Use NodeSource repository (Option A in Step 4) or nvm (Option C)

### Python Import Errors

**Problem**: `ModuleNotFoundError: No module named 'rclpy'`
**Solution**: Make sure ROS2 is sourced:
```bash
source /opt/ros/humble/setup.bash
```

### Permission Errors

**Problem**: Permission denied when installing packages
**Solution**: 
- Use `sudo` for system packages: `sudo apt install ...`
- Don't use `sudo` for pip/npm (use `--user` flag if needed)
- For project-specific packages, use a virtual environment

### Port 8000 Already in Use

**Problem**: `Address already in use` when starting backend
**Solution**:
```bash
# Find process using port 8000
sudo lsof -i :8000
# Kill it
sudo killall uvicorn
# Or kill specific PID
sudo kill <PID>
```

## Verification Checklist

After setup, verify:

- [ ] Python 3 installed: `python3 --version`
- [ ] pip installed: `pip3 --version`
- [ ] Node.js installed: `node --version` (should be v16+)
- [ ] npm installed: `npm --version`
- [ ] ROS2 installed: `ros2 --help` works
- [ ] rclpy importable: `python3 -c "import rclpy"`
- [ ] FastAPI installable: `pip install fastapi` works
- [ ] Frontend dependencies installable: `cd frontend && npm install` works
- [ ] Backend starts: `uvicorn backend.main:app --host 0.0.0.0 --port 8000`
- [ ] Backend accessible: `curl http://localhost:8000/status`

## Next Steps

After completing this setup:

1. Follow **[ROBOT_SETUP.md](ROBOT_SETUP.md)** to configure and run your project
2. Follow **[CONNECTION_GUIDE.md](CONNECTION_GUIDE.md)** to connect your interface
3. Test the connection using `./test_connection.sh`

## Notes

- **ROS2 is large**: Installing ROS2 can take 30+ minutes and requires several GB of disk space
- **Use 64-bit OS**: For best compatibility, use Raspberry Pi OS 64-bit
- **SSD recommended**: If using an external SSD instead of SD card, you'll get better performance
- **Keep updated**: Regularly run `sudo apt update && sudo apt upgrade` to keep packages updated

