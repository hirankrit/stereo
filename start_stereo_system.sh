#!/bin/bash
# Start Complete Stereo Camera System
# Launches camera node and opens viewer

set -e

# Source ROS2
source /opt/ros/humble/setup.bash

echo "=========================================="
echo "Starting Stereo Camera System"
echo "=========================================="
echo ""

# Stop any existing nodes
echo "1. Stopping existing camera nodes..."
pkill -f gstreamer_camera_node || true
sleep 2

# Start camera node in background
echo "2. Starting Stereo Camera Node..."
python3 /home/jay/projects/stereo_camera/gstreamer_camera_node.py > /tmp/stereo_node.log 2>&1 &
CAMERA_PID=$!

echo "   Camera node started (PID: $CAMERA_PID)"
echo "   Logs: /tmp/stereo_node.log"

# Wait for node to initialize
echo "3. Waiting for cameras to initialize..."
sleep 5

# Check if topics are available
echo "4. Checking ROS2 topics..."
ros2 topic list | grep stereo || {
    echo "ERROR: Stereo topics not found!"
    echo "Check logs: tail -f /tmp/stereo_node.log"
    exit 1
}

echo ""
echo "✓ System ready!"
echo ""
echo "Available topics:"
ros2 topic list | grep stereo

echo ""
echo "5. Opening camera viewer..."
echo ""
echo "=========================================="
echo "Stereo Camera System Running"
echo "=========================================="
echo ""
echo "To view cameras:"
echo "  - Select topic from dropdown menu"
echo "  - /stereo/left/image_raw  = Left camera"
echo "  - /stereo/right/image_raw = Right camera"
echo ""
echo "To stop:"
echo "  - Close viewer window"
echo "  - Run: pkill -f gstreamer_camera_node"
echo ""
echo "Logs: tail -f /tmp/stereo_node.log"
echo "=========================================="
echo ""

# Launch viewer
ros2 run rqt_image_view rqt_image_view
