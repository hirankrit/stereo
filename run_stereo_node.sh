#!/bin/bash
# Run Stereo Camera Node
# Automatically kills existing nodes and starts fresh

set -e

echo "Stopping any existing camera nodes..."
pkill -f gstreamer_camera_node || true

echo "Waiting for cameras to be released..."
sleep 2

echo "Starting Stereo Camera Node..."
source /opt/ros/humble/setup.bash
python3 /home/jay/projects/stereo_camera/gstreamer_camera_node.py
