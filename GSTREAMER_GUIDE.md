# GStreamer Camera Setup Guide for Jetson

## Why GStreamer instead of V4L2?

IMX219 CSI cameras on Jetson **require** NVIDIA's Argus API:
- ✅ **nvarguscamerasrc** - Native Jetson driver (GStreamer)
- ❌ **V4L2** - Does NOT work with IMX219 CSI cameras
- ❌ **Isaac ROS Argus** - Requires complex GXF dependencies

**Solution**: Custom ROS2 node using GStreamer + nvarguscamerasrc + OpenCV

## Quick Start

### Step 1: Run Setup Script (in container)

```bash
bash /workspace/setup_gstreamer_cameras.sh
```

This will:
1. Install Python OpenCV and GStreamer dependencies
2. Test nvarguscamerasrc availability
3. Test both cameras with GStreamer
4. Test OpenCV + GStreamer integration
5. Test ROS2 camera node

### Step 2: Test Single Camera

```bash
source /opt/ros/humble/setup.bash
python3 /workspace/gstreamer_camera_node.py --ros-args -p sensor_id:=0
```

In another terminal:
```bash
source /opt/ros/humble/setup.bash
ros2 topic list
ros2 topic hz /image_raw
ros2 topic echo /camera_info --once
```

### Step 3: Run Both Cameras (Stereo Mode)

**Option A: Using two terminals**

Terminal 1 (Left camera):
```bash
source /opt/ros/humble/setup.bash
python3 /workspace/gstreamer_camera_node.py \
  --ros-args \
  -p sensor_id:=0 \
  -p camera_name:=left_camera \
  -r image_raw:=/stereo/left/image_raw \
  -r camera_info:=/stereo/left/camera_info
```

Terminal 2 (Right camera):
```bash
source /opt/ros/humble/setup.bash
python3 /workspace/gstreamer_camera_node.py \
  --ros-args \
  -p sensor_id:=1 \
  -p camera_name:=right_camera \
  -r image_raw:=/stereo/right/image_raw \
  -r camera_info:=/stereo/right/camera_info
```

**Option B: Using launch file (if colcon workspace is setup)**
```bash
ros2 launch /workspace/launch/stereo_gstreamer.launch.py
```

### Step 4: Verify Topics

```bash
# List topics
ros2 topic list | grep stereo

# Expected output:
# /stereo/left/camera_info
# /stereo/left/image_raw
# /stereo/right/camera_info
# /stereo/right/image_raw

# Check frame rate
ros2 topic hz /stereo/left/image_raw
ros2 topic hz /stereo/right/image_raw

# Should show ~30 Hz for both
```

## How It Works

### GStreamer Pipeline

```
nvarguscamerasrc sensor-id=0
  ↓
video/x-raw(memory:NVMM) - NVIDIA Memory (GPU)
  ↓
nvvidconv - NVIDIA Video Converter
  ↓
video/x-raw, format=BGRx
  ↓
videoconvert - Format conversion
  ↓
video/x-raw, format=BGR - OpenCV compatible
  ↓
appsink - Application sink (OpenCV reads here)
```

### ROS2 Node Architecture

1. **gstreamer_camera_node.py**
   - Opens camera with `cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)`
   - Captures frames at specified rate
   - Publishes to ROS2 topics:
     - `image_raw` - sensor_msgs/Image
     - `camera_info` - sensor_msgs/CameraInfo

2. **Parameters**:
   - `sensor_id` - Camera sensor (0 or 1)
   - `width` - Image width (default: 1280)
   - `height` - Image height (default: 720)
   - `framerate` - FPS (default: 30)
   - `flip_method` - Rotation (0-5)
   - `camera_name` - Camera identifier

## Troubleshooting

### Issue: nvarguscamerasrc not found

The container might not have access to Jetson's Argus libraries.

**Solution**: Mount more host libraries to container:
```bash
# Update run_ros2_container.sh to add:
--volume /usr/lib/aarch64-linux-gnu/tegra:/usr/lib/aarch64-linux-gnu/tegra:ro \
--volume /etc/nv_tegra_release:/etc/nv_tegra_release:ro
```

### Issue: Camera fails to open

Check if cameras work on host:
```bash
# Exit container, run on host:
gst-launch-1.0 nvarguscamerasrc sensor-id=0 ! fakesink
```

If this works on host but not in container, we need to run the node on the **host** instead of container.

### Issue: Permission denied

Make sure container runs with `--privileged` and has device access:
```bash
--device /dev/video0:/dev/video0
--device /dev/video1:/dev/video1
```

## Alternative: Run on Host Instead of Container

If nvarguscamerasrc doesn't work in container:

**Install ROS2 Humble on host Jetson:**
```bash
# Add ROS2 repository (ARM64)
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update

# Install ROS2 Humble from source or use NVIDIA's pre-built image
# See: https://docs.ros.org/en/humble/Installation.html
```

**Then run the camera node directly on host:**
```bash
python3 /home/jay/projects/stereo_camera/gstreamer_camera_node.py \
  --ros-args -p sensor_id:=0
```

## Camera Resolutions

IMX219 supported modes:
- **1920x1080 @ 30 fps** - Full HD
- **1280x720 @ 60 fps** - HD (default)
- **640x480 @ 90 fps** - VGA

Change in launch parameters:
```bash
python3 /workspace/gstreamer_camera_node.py \
  --ros-args \
  -p sensor_id:=0 \
  -p width:=1920 \
  -p height:=1080 \
  -p framerate:=30
```

## Next Steps

1. ✅ Get both cameras publishing to ROS2
2. Check timestamp synchronization
3. Setup IMU (ICM20948) ROS2 node
4. Camera + IMU calibration with Kalibr
5. Stereo rectification and depth mapping
6. Object detection for crop harvesting

## Files Created

- `gstreamer_camera_node.py` - ROS2 camera node
- `launch/stereo_gstreamer.launch.py` - Launch file
- `setup_gstreamer_cameras.sh` - Setup script
- `GSTREAMER_GUIDE.md` - This guide

Good luck! 🚀
