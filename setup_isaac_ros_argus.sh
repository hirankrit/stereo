#!/bin/bash
# Isaac ROS Argus Camera Setup Script
# Run this inside the ROS2 container

set -e

echo "=========================================="
echo "Isaac ROS Argus Camera Setup"
echo "=========================================="

# Source ROS2
source /opt/ros/humble/setup.bash

# Install dependencies
echo "Installing dependencies..."
apt update
apt install -y \
    python3-pip \
    git \
    cmake \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool

# Create workspace
echo "Creating Isaac ROS workspace..."
mkdir -p ~/isaac_ros_ws/src
cd ~/isaac_ros_ws/src

# Clone Isaac ROS Common (dependency)
echo "Cloning Isaac ROS Common..."
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git

# Clone Isaac ROS Argus Camera
echo "Cloning Isaac ROS Argus Camera..."
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_argus_camera.git

# Install rosdep dependencies
cd ~/isaac_ros_ws
echo "Installing rosdep dependencies..."
rosdep init || true
rosdep update
rosdep install -y --from-paths src --ignore-src --rosdistro humble

# Build the workspace
echo "Building workspace..."
cd ~/isaac_ros_ws
colcon build --symlink-install

echo ""
echo "=========================================="
echo "Installation complete!"
echo "=========================================="
echo ""
echo "To use isaac_ros_argus_camera, run:"
echo "  source ~/isaac_ros_ws/install/setup.bash"
echo ""
