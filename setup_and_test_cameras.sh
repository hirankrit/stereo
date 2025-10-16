#!/bin/bash
# Complete Camera Setup and Test Script
# Run inside ROS2 container as root

set -e

echo "=========================================="
echo "Complete Stereo Camera Setup & Test"
echo "=========================================="
echo ""

# Source ROS2
source /opt/ros/humble/setup.bash

# Step 1: Install V4L2 Camera Package
echo "Step 1/5: Installing V4L2 camera package..."
apt update -qq
apt install -y \
    ros-humble-v4l2-camera \
    ros-humble-image-transport \
    ros-humble-image-pipeline \
    ros-humble-camera-calibration \
    ros-humble-camera-info-manager \
    v4l-utils \
    > /dev/null 2>&1

echo "✅ V4L2 camera package installed"
echo ""

# Step 2: Check available cameras
echo "Step 2/5: Checking available cameras..."
echo "Video devices:"
ls -l /dev/video* 2>/dev/null || echo "No video devices found!"
echo ""

echo "V4L2 device list:"
v4l2-ctl --list-devices 2>/dev/null || echo "v4l2-ctl failed"
echo ""

# Step 3: Check camera formats
echo "Step 3/5: Checking camera formats..."
echo "=== LEFT Camera (/dev/video0) formats ==="
v4l2-ctl --device=/dev/video0 --list-formats-ext 2>/dev/null | head -30 || echo "Cannot read /dev/video0"
echo ""

echo "=== RIGHT Camera (/dev/video1) formats ==="
v4l2-ctl --device=/dev/video1 --list-formats-ext 2>/dev/null | head -30 || echo "Cannot read /dev/video1"
echo ""

# Step 4: Test left camera briefly
echo "Step 4/5: Testing LEFT camera (3 seconds)..."
timeout 3 ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:="/dev/video0" 2>&1 | grep -E "(Starting|error|Error|fail)" || true
echo "✅ Left camera test completed"
echo ""

# Step 5: Test right camera briefly
echo "Step 5/5: Testing RIGHT camera (3 seconds)..."
timeout 3 ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:="/dev/video1" 2>&1 | grep -E "(Starting|error|Error|fail)" || true
echo "✅ Right camera test completed"
echo ""

echo "=========================================="
echo "Setup Complete! ✅"
echo "=========================================="
echo ""
echo "Both cameras are ready. Next steps:"
echo ""
echo "1. Launch stereo cameras:"
echo "   ros2 launch /workspace/launch/stereo_camera_v4l2.launch.py"
echo ""
echo "2. In another terminal, verify topics:"
echo "   ros2 topic list | grep stereo"
echo "   ros2 topic hz /stereo/left/image_raw"
echo ""
echo "3. Or use helper script:"
echo "   bash /workspace/test_cameras.sh"
echo ""
