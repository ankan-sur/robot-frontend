#!/bin/bash

# Startup script for running the robot backend
# Run this on the robot after copying the project

set -e

echo "Starting HFH Robot Backend..."

# Get the directory where the script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

# Check if virtual environment exists, create if not
if [ ! -d "venv" ]; then
    echo "Creating virtual environment..."
    python3 -m venv venv
fi

# Activate virtual environment
echo "Activating virtual environment..."
source venv/bin/activate

# Install/update dependencies
echo "Installing dependencies..."
pip install -r requirements.txt

# Check if ROS2 is available
if ! command -v ros2 &> /dev/null; then
    echo "Warning: ROS2 not found in PATH. Make sure ROS2 is installed and sourced."
    echo "Try: source /opt/ros/humble/setup.bash"
fi

# Set up environment variables if needed
export CONTROL_TOKEN=${CONTROL_TOKEN:-""}

# Get the robot's IP address
ROBOT_IP=$(hostname -I | awk '{print $1}')
echo "Robot IP: $ROBOT_IP"
echo "Backend will be available at: http://$ROBOT_IP:8000"
echo "API docs will be available at: http://$ROBOT_IP:8000/docs"
echo ""
echo "Starting backend server..."
echo "Press Ctrl+C to stop"
echo ""

# Run the backend
uvicorn backend.main:app --host 0.0.0.0 --port 8000

