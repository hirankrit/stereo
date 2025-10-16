#!/bin/bash
# V4L2 Camera Setup Script (Simpler alternative to Isaac ROS)
# Run this inside the ROS2 container

set -e

echo "=========================================="
echo "V4L2 Camera Setup for ROS2"
echo "=========================================="

# Source ROS2
source /opt/ros/humble/setup.bash

# Install dependencies
echo "Installing dependencies..."
apt update
apt install -y \
    ros-humble-v4l2-camera \
    ros-humble-image-transport \
    ros-humble-image-pipeline \
    ros-humble-camera-calibration \
    ros-humble-camera-info-manager \
    v4l-utils

echo ""
echo "=========================================="
echo "Installation complete!"
echo "=========================================="
echo ""
echo "V4L2 camera package installed successfully."
echo "You can now test the cameras with:"
echo "  ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:=/dev/video0"
echo ""
