#!/bin/bash
# GStreamer Camera Setup and Test Script
# Run inside ROS2 container

set -e

echo "=========================================="
echo "GStreamer Stereo Camera Setup"
echo "=========================================="
echo ""

# Source ROS2
source /opt/ros/humble/setup.bash

# Step 1: Install dependencies
echo "Step 1/6: Installing dependencies..."
apt update -qq
apt install -y \
    python3-opencv \
    python3-pip \
    ros-humble-cv-bridge \
    gstreamer1.0-tools \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    > /dev/null 2>&1

echo "✅ Dependencies installed"
echo ""

# Step 2: Test GStreamer with nvarguscamerasrc
echo "Step 2/6: Testing GStreamer pipeline availability..."

if gst-inspect-1.0 nvarguscamerasrc > /dev/null 2>&1; then
    echo "✅ nvarguscamerasrc plugin found"
else
    echo "❌ nvarguscamerasrc plugin NOT found"
    echo "This might be a container issue - nvarguscamerasrc requires host Jetson libraries"
fi
echo ""

# Step 3: Test left camera with GStreamer directly
echo "Step 3/6: Testing LEFT camera with GStreamer (5 seconds)..."
timeout 5 gst-launch-1.0 \
    nvarguscamerasrc sensor-id=0 ! \
    'video/x-raw(memory:NVMM), width=1280, height=720, framerate=30/1' ! \
    nvvidconv ! \
    'video/x-raw, format=BGRx' ! \
    videoconvert ! \
    'video/x-raw, format=BGR' ! \
    fakesink 2>&1 | grep -E "(Setting|ERROR|error)" || true
echo "✅ Left camera GStreamer test completed"
echo ""

# Step 4: Test right camera with GStreamer directly
echo "Step 4/6: Testing RIGHT camera with GStreamer (5 seconds)..."
timeout 5 gst-launch-1.0 \
    nvarguscamerasrc sensor-id=1 ! \
    'video/x-raw(memory:NVMM), width=1280, height=720, framerate=30/1' ! \
    nvvidconv ! \
    'video/x-raw, format=BGRx' ! \
    videoconvert ! \
    'video/x-raw, format=BGR' ! \
    fakesink 2>&1 | grep -E "(Setting|ERROR|error)" || true
echo "✅ Right camera GStreamer test completed"
echo ""

# Step 5: Test Python OpenCV with GStreamer
echo "Step 5/6: Testing Python OpenCV GStreamer integration..."
python3 << 'EOF'
import cv2
import sys

pipeline = (
    "nvarguscamerasrc sensor-id=0 ! "
    "video/x-raw(memory:NVMM), width=1280, height=720, framerate=30/1 ! "
    "nvvidconv ! "
    "video/x-raw, format=BGRx ! "
    "videoconvert ! "
    "video/x-raw, format=BGR ! "
    "appsink"
)

print("Opening camera with OpenCV...")
cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

if cap.isOpened():
    print("✅ OpenCV can open GStreamer pipeline")
    ret, frame = cap.read()
    if ret:
        print(f"✅ Frame captured: {frame.shape}")
    else:
        print("❌ Failed to capture frame")
    cap.release()
else:
    print("❌ Failed to open camera with OpenCV")
    sys.exit(1)
EOF

echo ""

# Step 6: Make camera node executable and test
echo "Step 6/6: Testing ROS2 camera node..."
chmod +x /workspace/gstreamer_camera_node.py

echo "Testing left camera node (10 seconds)..."
timeout 10 python3 /workspace/gstreamer_camera_node.py \
    --ros-args \
    -p sensor_id:=0 \
    -p camera_name:=left_camera \
    2>&1 | head -20 || true

echo ""
echo "=========================================="
echo "Setup Complete! ✅"
echo "=========================================="
echo ""
echo "Next steps:"
echo ""
echo "1. Test single camera:"
echo "   python3 /workspace/gstreamer_camera_node.py --ros-args -p sensor_id:=0"
echo ""
echo "2. Launch both cameras:"
echo "   ros2 run --prefix 'python3 ' launch /workspace/launch/stereo_gstreamer.launch.py"
echo ""
echo "3. Or run manually:"
echo "   # Terminal 1:"
echo "   python3 /workspace/gstreamer_camera_node.py --ros-args -p sensor_id:=0 -p camera_name:=left_camera -r image_raw:=/stereo/left/image_raw"
echo ""
echo "   # Terminal 2:"
echo "   python3 /workspace/gstreamer_camera_node.py --ros-args -p sensor_id:=1 -p camera_name:=right_camera -r image_raw:=/stereo/right/image_raw"
echo ""
echo "4. Check topics:"
echo "   ros2 topic list | grep stereo"
echo "   ros2 topic hz /stereo/left/image_raw"
echo ""
