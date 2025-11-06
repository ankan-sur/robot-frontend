#!/bin/bash

# Fresh Raspberry Pi Setup Script
# This script installs Node.js, npm, Python, and essential tools
# ROS2 must be installed separately (see RASPBERRY_PI_SETUP.md)

set -e

echo "=========================================="
echo "Fresh Raspberry Pi Setup Script"
echo "=========================================="
echo ""

# Check if running on Raspberry Pi
if [ ! -f /proc/device-tree/model ]; then
    echo "Warning: This doesn't appear to be a Raspberry Pi"
    read -p "Continue anyway? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# Update system
echo "=== Step 1: Updating system packages ==="
sudo apt update
sudo apt upgrade -y
echo "✓ System updated"
echo ""

# Install essential build tools
echo "=== Step 2: Installing essential build tools ==="
sudo apt install -y \
    build-essential \
    git \
    curl \
    wget \
    vim \
    software-properties-common \
    locales
echo "✓ Build tools installed"
echo ""

# Install Python
echo "=== Step 3: Installing Python and pip ==="
sudo apt install -y python3 python3-pip python3-venv python3-dev
pip3 install --upgrade pip
echo "✓ Python installed: $(python3 --version)"
echo "✓ pip installed: $(pip3 --version)"
echo ""

# Install Node.js and npm
echo "=== Step 4: Installing Node.js and npm ==="
echo "Installing Node.js 20.x LTS from NodeSource..."

# Add NodeSource repository
curl -fsSL https://deb.nodesource.com/setup_20.x | sudo -E bash -

# Install Node.js
sudo apt install -y nodejs

echo "✓ Node.js installed: $(node --version)"
echo "✓ npm installed: $(npm --version)"
echo ""

# Install additional Python dependencies
echo "=== Step 5: Installing additional Python dependencies ==="
sudo apt install -y \
    libffi-dev \
    libssl-dev \
    python3-setuptools \
    python3-wheel \
    python3-colcon-common-extensions
echo "✓ Additional dependencies installed"
echo ""

# Set up locale (needed for ROS2)
echo "=== Step 6: Setting up locale ==="
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
echo "✓ Locale configured"
echo ""

# Summary
echo "=========================================="
echo "Setup Complete!"
echo "=========================================="
echo ""
echo "Installed versions:"
echo "  Python: $(python3 --version)"
echo "  pip: $(pip3 --version)"
echo "  Node.js: $(node --version)"
echo "  npm: $(npm --version)"
echo ""
echo "Next steps:"
echo "1. Install ROS2 Humble (see RASPBERRY_PI_SETUP.md Step 5)"
echo "2. Copy your project to the Pi"
echo "3. Install project dependencies:"
echo "   cd ~/robot_project"
echo "   python3 -m venv venv"
echo "   source venv/bin/activate"
echo "   pip install -r requirements.txt"
echo "4. Install frontend dependencies:"
echo "   cd ~/robot_project/frontend"
echo "   npm install"
echo ""
echo "For detailed instructions, see RASPBERRY_PI_SETUP.md"
echo ""

