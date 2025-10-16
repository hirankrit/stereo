# Isaac ROS Argus Camera Setup Guide

## Overview
This guide will help you set up the Isaac ROS Argus Camera driver for your stereo camera setup on Jetson Orin Nano.

## Prerequisites
- You must be inside the ROS2 container
- Run: `./run_ros2_container.sh` to start the container

## Step 1: Install Isaac ROS Argus Camera

Inside the container, run:

```bash
cd /workspace
bash setup_isaac_ros_argus.sh
```

This script will:
- Install all required dependencies
- Clone Isaac ROS Common and Isaac ROS Argus Camera repositories
- Build the workspace with colcon
- Takes approximately 10-15 minutes

## Step 2: Source the Workspace

After installation completes:

```bash
source ~/isaac_ros_ws/install/setup.bash
```

**Important**: You need to source this in every new terminal/shell session.

## Step 3: Test Individual Cameras

### Test Left Camera (sensor_id=0):
```bash
source /opt/ros/humble/setup.bash
source ~/isaac_ros_ws/install/setup.bash

ros2 run isaac_ros_argus_camera isaac_ros_argus_camera \
  --ros-args \
  -p sensor_id:=0 \
  -p camera_id:=0 \
  -p module_id:=0 \
  -p framerate:=30 \
  -p width:=1280 \
  -p height:=720
```

In another terminal (inside container), check the topics:
```bash
ros2 topic list
ros2 topic echo /image_raw --once
ros2 topic hz /image_raw
```

### Test Right Camera (sensor_id=1):
```bash
ros2 run isaac_ros_argus_camera isaac_ros_argus_camera \
  --ros-args \
  -p sensor_id:=1 \
  -p camera_id:=1 \
  -p module_id:=1 \
  -p framerate:=30 \
  -p width:=1280 \
  -p height:=720
```

## Step 4: Launch Stereo Camera System

Once individual cameras work, launch both simultaneously:

```bash
source /opt/ros/humble/setup.bash
source ~/isaac_ros_ws/install/setup.bash

ros2 launch /workspace/launch/stereo_camera.launch.py
```

### Verify Stereo Topics:
```bash
ros2 topic list | grep stereo
```

Expected output:
```
/stereo/left/camera_info
/stereo/left/image_raw
/stereo/right/camera_info
/stereo/right/image_raw
```

### Check Frame Rate:
```bash
ros2 topic hz /stereo/left/image_raw
ros2 topic hz /stereo/right/image_raw
```

Should show ~30 Hz for both.

## Step 5: View Images (Optional)

If you have a display connected:

```bash
# Install rqt_image_view if needed
apt install -y ros-humble-rqt-image-view

# View left camera
ros2 run rqt_image_view rqt_image_view /stereo/left/image_raw

# View right camera (in another terminal)
ros2 run rqt_image_view rqt_image_view /stereo/right/image_raw
```

## Troubleshooting

### Camera Not Detected
Check if cameras are visible:
```bash
ls -l /dev/video*
v4l2-ctl --list-devices
```

### Permission Issues
Make sure container has privileged access (already configured in `run_ros2_container.sh`).

### Build Errors
If you encounter build errors during setup:
```bash
cd ~/isaac_ros_ws
colcon build --symlink-install --packages-select isaac_ros_argus_camera
```

### Check Argus Driver
Verify Argus is working at system level:
```bash
gst-launch-1.0 nvarguscamerasrc sensor-id=0 ! 'video/x-raw(memory:NVMM),width=1280,height=720,framerate=30/1' ! nvvidconv ! xvimagesink
```

## Hardware Configuration

- **Left Camera**: /dev/video0 → sensor_id=0
- **Right Camera**: /dev/video1 → sensor_id=1
- **Baseline**: 60mm (6cm between cameras)
- **Sensor**: IMX219 (both cameras)
- **Resolution**: 1280x720 @ 30fps

## Next Steps

After cameras are working:
1. Setup IMU driver (ICM20948)
2. Time synchronization between cameras and IMU
3. Camera-IMU calibration with Kalibr
4. Stereo rectification and depth mapping

## Launch File Parameters

You can customize the launch file:

```bash
ros2 launch /workspace/launch/stereo_camera.launch.py \
  frame_rate:=30 \
  width:=1280 \
  height:=720
```

Available resolutions for IMX219:
- 1920x1080 @ 30fps
- 1280x720 @ 60fps
- 640x480 @ 90fps

## Saving the Setup

To make sourcing automatic, add to container's bashrc:
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/isaac_ros_ws/install/setup.bash" >> ~/.bashrc
```

## Resources

- Isaac ROS Argus Camera: https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_argus_camera
- Isaac ROS Documentation: https://nvidia-isaac-ros.github.io/
- ROS2 Humble Docs: https://docs.ros.org/en/humble/
