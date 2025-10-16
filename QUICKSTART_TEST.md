# Quick Start - Stereo Camera Test

## ✅ สิ่งที่ทดสอบสำเร็จแล้ว

### 1. GStreamer Test (Host)
```bash
# Left camera
gst-launch-1.0 nvarguscamerasrc sensor-id=0 num-buffers=5 ! \
  'video/x-raw(memory:NVMM),width=1280,height=720,framerate=30/1' ! \
  nvvidconv ! fakesink

# Right camera  
gst-launch-1.0 nvarguscamerasrc sensor-id=1 num-buffers=5 ! \
  'video/x-raw(memory:NVMM),width=1280,height=720,framerate=30/1' ! \
  nvvidconv ! fakesink
```
**Result**: ✅ Both cameras working!

### 2. Python + OpenCV Test (Host)
```bash
python3 test_argus_simple.py
```
**Result**: ✅ Captured 10 stereo frame pairs!

---

## 🚀 Next: Test in ROS2 Container

### Option 1: Quick Test (arm64v8/ros:humble-ros-base)
Already downloaded! Smaller image (718MB)

```bash
# Start container
docker run -it --rm \
  --name ros2_quick_test \
  --runtime nvidia \
  --network host \
  --volume /tmp/argus_socket:/tmp/argus_socket \
  --volume /home/jay/projects/stereo_camera:/workspace \
  --volume /dev:/dev \
  --workdir /workspace \
  --env DISPLAY=$DISPLAY \
  arm64v8/ros:humble-ros-base \
  bash

# Inside container:
# 1. Test GStreamer
gst-launch-1.0 nvarguscamerasrc sensor-id=0 num-buffers=5 ! \
  'video/x-raw(memory:NVMM),width=1280,height=720,framerate=30/1' ! \
  nvvidconv ! fakesink

# 2. Install dependencies
apt-get update
apt-get install -y python3-opencv ros-humble-cv-bridge

# 3. Test Python
python3 test_argus_simple.py

# 4. Test ROS2 node
python3 gstreamer_camera_node.py
```

### Option 2: Full Test (dustynv/ros:humble-ros-base-l4t-r36.2.0)
Better for production! (~5-6 GB, download in background)

```bash
./start_ros2_stereo.sh
```

---

## 📊 Current Status

- ✅ Phase 1.1: Camera hardware verified
- ✅ Phase 1.2: GStreamer working
- ✅ Phase 1.3: Python + OpenCV working
- ⏳ Phase 1.4: ROS2 container test (in progress)
- ⏳ Phase 1.5: ROS2 node publishing test

**Hardware**: Both IMX219 cameras working perfectly  
**Software**: Python + OpenCV + GStreamer confirmed  
**Next**: ROS2 integration test

---

Created: 2025-10-16
