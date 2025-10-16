# Stereo Camera Quick Start Guide

## Problem with Isaac ROS Argus

Isaac ROS Argus Camera requires the full Isaac ROS/GXF infrastructure which has complex dependencies:
- `gxf_isaac_messages`
- `gxf_isaac_timestamp_correlator`
- Requires NVIDIA Isaac ROS base image

## Solution: Use V4L2 Camera (Recommended for now)

V4L2 is simpler, well-supported, and works immediately with standard ROS2.

### Step 1: Install V4L2 Camera Package

Inside the container:

```bash
cd /workspace
bash setup_v4l2_camera.sh
```

This installs:
- `ros-humble-v4l2-camera` - Camera driver
- Image transport and calibration tools
- V4L utilities

### Step 2: Test Individual Cameras

Check what formats are available:

```bash
v4l2-ctl --device=/dev/video0 --list-formats-ext
v4l2-ctl --device=/dev/video1 --list-formats-ext
```

**Easy way - Use helper script:**
```bash
bash /workspace/test_cameras.sh
# Select option 1 for left camera
```

**Manual command (NO line breaks!):**
```bash
source /opt/ros/humble/setup.bash
ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:="/dev/video0"
```

**IMPORTANT**: Type the entire command on ONE line with quotes around the device path.

In another terminal (inside container):
```bash
source /opt/ros/humble/setup.bash
ros2 topic list
ros2 topic hz /image_raw
ros2 topic echo /camera_info --once
```

Press Ctrl+C to stop, then test right camera:
```bash
ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:=/dev/video1
```

### Step 3: Launch Stereo System

Launch both cameras simultaneously:

```bash
source /opt/ros/humble/setup.bash
ros2 launch /workspace/launch/stereo_camera_v4l2.launch.py
```

### Step 4: Verify Topics

In another terminal:
```bash
source /opt/ros/humble/setup.bash

# List stereo topics
ros2 topic list | grep stereo

# Check frame rates
ros2 topic hz /stereo/left/image_raw
ros2 topic hz /stereo/right/image_raw

# Check camera info
ros2 topic echo /stereo/left/camera_info --once
ros2 topic echo /stereo/right/camera_info --once
```

Expected topics:
```
/stereo/left/image_raw
/stereo/left/camera_info
/stereo/right/image_raw
/stereo/right/camera_info
```

### Step 5: View Images (Optional)

If you have X11/display:

```bash
apt install -y ros-humble-rqt-image-view

# View left camera
ros2 run rqt_image_view rqt_image_view /stereo/left/image_raw

# Or use image_view
apt install -y ros-humble-image-view
ros2 run image_view image_view --ros-args -r image:=/stereo/left/image_raw
```

## Launch Parameters

Customize resolution and format:

```bash
ros2 launch /workspace/launch/stereo_camera_v4l2.launch.py \
  image_width:=1280 \
  image_height:=720 \
  framerate:=30 \
  pixel_format:=YUYV
```

Common formats for IMX219:
- `YUYV` - Uncompressed (lower CPU usage)
- `MJPEG` - Compressed (higher CPU usage, less bandwidth)

## Troubleshooting

### Check Camera Devices
```bash
ls -l /dev/video*
v4l2-ctl --list-devices
```

### Check Available Formats
```bash
v4l2-ctl --device=/dev/video0 --list-formats-ext
```

### Permission Issues
Container should run with `--privileged` flag (already configured in `run_ros2_container.sh`).

### Camera Not Opening
Try resetting the camera:
```bash
# Check if any process is using the camera
lsof /dev/video0
lsof /dev/video1

# If needed, kill the process or restart container
```

## Performance Notes

**V4L2 vs Isaac ROS Argus:**

| Feature | V4L2 Camera | Isaac ROS Argus |
|---------|-------------|-----------------|
| Setup | Simple ✅ | Complex ❌ |
| CPU Usage | Moderate | Lower (GPU accelerated) |
| Latency | Normal | Lower |
| Jetson Optimization | No | Yes |
| Hardware Sync | No | Yes (better for stereo) |

**Recommendation:** Start with V4L2 for development, switch to Isaac ROS Argus later for production.

## Next Steps

1. ✅ Get cameras working with V4L2
2. Test synchronized capture
3. Setup IMU driver (ICM20948)
4. Camera calibration with Kalibr
5. Consider Isaac ROS Argus later if you need:
   - Hardware timestamp synchronization
   - GPU-accelerated image processing
   - Lower latency

## Alternative: Try Isaac ROS with Full Container

If you want to try Isaac ROS properly, you need their full Docker image:

```bash
# Pull NVIDIA Isaac ROS base image (requires NGC login)
docker pull nvcr.io/nvidia/isaac/ros:humble-aarch64

# Or use their dev container setup
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git
cd isaac_ros_common
# Follow their container setup instructions
```

But for now, V4L2 is the practical choice! 🚀
