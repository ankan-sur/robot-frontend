#!/bin/bash

# Quick connection test script
# Run this to verify you can connect to the robot backend

ROBOT_IP=${1:-"192.168.149.1"}
BACKEND_URL="http://${ROBOT_IP}:8000"

echo "Testing connection to robot backend at ${BACKEND_URL}..."
echo ""

# Test 1: Check if backend is reachable
echo "1. Testing backend reachability..."
if curl -s --connect-timeout 5 "${BACKEND_URL}/status" > /dev/null; then
    echo "   ✓ Backend is reachable"
else
    echo "   ✗ Cannot reach backend at ${BACKEND_URL}"
    echo "   Make sure:"
    echo "   - Robot is powered on"
    echo "   - You're connected to 'Fordward' WiFi (password: fordward)"
    echo "   - Backend is running on the robot"
    exit 1
fi

# Test 2: Get status
echo ""
echo "2. Getting robot status..."
STATUS=$(curl -s "${BACKEND_URL}/status")
if [ $? -eq 0 ]; then
    echo "   ✓ Status retrieved successfully:"
    echo "$STATUS" | python3 -m json.tool 2>/dev/null || echo "$STATUS"
else
    echo "   ✗ Failed to get status"
    exit 1
fi

# Test 3: Check API docs
echo ""
echo "3. Checking API documentation..."
if curl -s --connect-timeout 5 "${BACKEND_URL}/docs" > /dev/null; then
    echo "   ✓ API docs available at ${BACKEND_URL}/docs"
else
    echo "   ✗ API docs not available"
fi

echo ""
echo "Connection test complete!"
echo ""
echo "To use the frontend:"
echo "1. Create frontend/.env file with: VITE_API_BASE=${BACKEND_URL}"
echo "2. Run: cd frontend && npm run dev"
echo "3. Open: http://localhost:5173"

