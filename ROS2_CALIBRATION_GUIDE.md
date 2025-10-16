# ROS2 Stereo Camera Calibration Guide

**Date**: 2025-10-16
**System**: Jetson Orin Nano + IMX219 Stereo Camera
**ROS2 Version**: Humble
**Method**: camera_calibration package (camera-only, no IMU)

---

## Overview

This guide explains how to calibrate the stereo camera system using ROS2's `camera_calibration` package. This calibration is necessary before using stereo processing features like depth maps.

**Note**: This is a **camera-only calibration**. IMU integration can be added later if hardware becomes available (see `IMU_INVESTIGATION.md`).

---

## Prerequisites

### 1. Installed Software

Ensure these packages are installed:
```bash
# Check if packages are installed
dpkg -l | grep ros-humble-camera-calibration
dpkg -l | grep ros-humble-image-pipeline
dpkg -l | grep ros-humble-stereo-image-proc
```

If not installed, run:
```bash
sudo ./install_ros2_calibration.sh
```

### 2. Calibration Target

You need a calibration pattern. Two options:

**Option A: Chessboard Pattern (Recommended - you already have one!)**
- Size: 7×7 squares (6×6 inner corners)
- Square size: 25mm × 25mm
- Print on rigid, flat surface
- Ensure good contrast (black and white)

**Option B: April Tags** (Alternative)
- More robust detection in poor lighting
- Can handle partial occlusion
- Requires separate setup (see April Tags section)

### 3. Environment Setup

- **Lighting**: Good, uniform lighting (avoid shadows)
- **Space**: Enough room to move calibration board around (1-2 meters)
- **Board mounting**: Flat, rigid board (no warping/bending)

---

## Calibration Process

### Step 1: Start Stereo Camera Node

Open **Terminal 1**:
```bash
cd /home/jay/projects/stereo_camera
source /opt/ros/humble/setup.bash
./run_stereo_node.sh
```

Verify topics are publishing:
```bash
# In another terminal
source /opt/ros/humble/setup.bash
ros2 topic list | grep stereo
```

Expected output:
```
/stereo/left/camera_info
/stereo/left/image_raw
/stereo/right/camera_info
/stereo/right/image_raw
```

### Step 2: Launch Calibration Tool

Open **Terminal 2**:
```bash
cd /home/jay/projects/stereo_camera
source /opt/ros/humble/setup.bash

# For stereo calibration with chessboard (6x6 inner corners, 25mm squares)
ros2 run camera_calibration cameracalibrator \
  --size 6x6 \
  --square 0.025 \
  --approximate 0.1 \
  --no-service-check \
  right:=/stereo/right/image_raw \
  left:=/stereo/left/image_raw \
  right_camera:=/stereo/right \
  left_camera:=/stereo/left
```

**Parameters explained**:
- `--size 6x6`: Number of **inner corners** (not squares!)
- `--square 0.025`: Square size in **meters** (25mm = 0.025m)
- `--approximate 0.1`: Time synchronization tolerance (100ms)
- `--no-service-check`: Skip service availability check
- Topic remapping: Maps topics to calibrator's expected names

### Step 3: Collect Calibration Data

A calibration window will appear showing both camera views.

**Calibration Buttons** (initially grayed out):
- `CALIBRATE` - Compute calibration (appears when enough data collected)
- `SAVE` - Save calibration results
- `COMMIT` - Save to camera (not used for our setup)

**Progress Bars** (must all turn green):
- **X**: Left-right movement coverage
- **Y**: Up-down movement coverage
- **Size**: Depth/distance coverage (near-far)
- **Skew**: Tilted angle coverage

**How to collect data**:

1. **Hold chessboard** in view of both cameras
2. **Wait for detection**: Board will be highlighted when detected
3. **Move board slowly** through these positions:
   - Top-left, top-right, bottom-left, bottom-right corners
   - Center of frame
   - Close to camera (large in frame)
   - Far from camera (small in frame)
   - Tilted at various angles
   - Rotated (portrait/landscape)

4. **Watch progress bars**: All must reach the right end and turn green
5. **Aim for 40-60 good samples** (shown in window title)

**Tips**:
- Move slowly and smoothly
- Keep board flat (no warping)
- Ensure both cameras see entire pattern
- Fill entire frame coverage (corners + edges + center)
- Vary distance (25cm - 100cm range)
- Tilt board at different angles

### Step 4: Compute Calibration

When all progress bars are green and you have 40+ samples:

1. Click **CALIBRATE** button
2. Wait for computation (may take 1-3 minutes)
3. Review reprojection error in terminal output
   - **Good**: < 0.5 pixels average
   - **Acceptable**: 0.5 - 1.0 pixels
   - **Poor**: > 1.0 pixels (consider recalibrating)

### Step 5: Save Calibration

1. Click **SAVE** button
2. Calibration saved to: `/tmp/calibrationdata.tar.gz`
3. Extract and copy to project:

```bash
cd /home/jay/projects/stereo_camera
mkdir -p calibration_ros2
cd calibration_ros2

# Extract calibration
tar -xzf /tmp/calibrationdata.tar.gz

# Files created:
#   - left.yaml   : Left camera calibration
#   - right.yaml  : Right camera calibration
#   - ost.yaml    : Combined calibration (OpenCV format)
```

4. **Backup calibration**:
```bash
cp /tmp/calibrationdata.tar.gz \
   /home/jay/projects/stereo_camera/calibration_ros2_$(date +%Y%m%d_%H%M%S).tar.gz
```

---

## Calibration File Format

Example `left.yaml`:
```yaml
image_width: 1280
image_height: 720
camera_name: narrow_stereo/left
camera_matrix:
  rows: 3
  cols: 3
  data: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: [k1, k2, t1, t2, k3]
rectification_matrix:
  rows: 3
  cols: 3
  data: [r11, r12, r13, r21, r22, r23, r31, r32, r33]
projection_matrix:
  rows: 3
  cols: 4
  data: [fx', 0, cx', Tx, 0, fy', cy', 0, 0, 0, 1, 0]
```

**Key parameters**:
- `camera_matrix`: Intrinsic parameters (focal length, optical center)
- `distortion_coefficients`: Lens distortion (radial + tangential)
- `rectification_matrix`: Rotation to align image planes
- `projection_matrix`: Includes baseline (Tx) for stereo

---

## Using Calibration with stereo_image_proc

### Launch Stereo Processing

Create launch file: `launch_stereo_proc.sh`

```bash
#!/bin/bash
source /opt/ros/humble/setup.bash

# Start stereo_image_proc node
ros2 run stereo_image_proc stereo_image_proc_node \
  --ros-args \
  -r left/image_raw:=/stereo/left/image_raw \
  -r right/image_raw:=/stereo/right/image_raw \
  -r left/camera_info:=/stereo/left/camera_info \
  -r right/camera_info:=/stereo/right/camera_info \
  -p approximate_sync:=true \
  -p queue_size:=10
```

### Published Topics

After launching stereo_image_proc:
```bash
ros2 topic list | grep stereo
```

New topics available:
```
/stereo/disparity          # Disparity map
/stereo/points2            # Point cloud (3D)
/stereo/left/image_rect    # Rectified left image
/stereo/right/image_rect   # Rectified right image
```

### View Results

**Disparity Map**:
```bash
ros2 run rqt_image_view rqt_image_view /stereo/disparity
```

**Point Cloud** (requires rviz2):
```bash
rviz2
# Add -> PointCloud2
# Topic: /stereo/points2
# Fixed Frame: camera_link or left_camera_frame
```

---

## Troubleshooting

### Problem: "CALIBRATE button stays grayed out"

**Solutions**:
1. Check all progress bars are green
2. Collect more samples (aim for 40-60)
3. Ensure pattern detected in **both cameras** simultaneously
4. Move board to fill missing areas (check which bars not green)

### Problem: "Pattern not detected"

**Solutions**:
1. **Lighting**: Ensure good, uniform lighting
2. **Focus**: Adjust distance if board is blurry
3. **Size**: Pattern may be too large/small in frame
4. **Contrast**: Ensure black/white contrast is high
5. **Flatness**: Check board is not warped/bent
6. **Entire pattern visible**: Both cameras must see all corners

### Problem: "High reprojection error (>1.0 pixels)"

**Solutions**:
1. **Recalibrate** with more careful data collection
2. **Lighting**: Improve lighting conditions
3. **Pattern quality**: Use higher quality print
4. **Board flatness**: Ensure perfectly flat surface
5. **More samples**: Collect 60+ samples covering all areas

### Problem: "Only one camera shows pattern detection"

**Solutions**:
1. **Distance**: Board may be too close/far for both cameras
2. **Angle**: Adjust angle so both cameras see entire pattern
3. **Lighting**: One camera may have different exposure
4. **Baseline**: Remember cameras are 60mm apart - account for parallax

### Problem: "Calibration window freezes during computation"

**Solutions**:
1. **Wait longer**: Computation can take 2-5 minutes
2. **Check terminal**: Look for progress messages or errors
3. **Memory**: Jetson may be running out of memory (close other apps)
4. **Restart**: Kill process and try again with fewer samples

---

## Verification

### Check Calibration Quality

1. **Reprojection Error**: Should be < 0.5 pixels (shown in terminal)

2. **Epipolar Lines**: Use rectified images to check alignment
   ```bash
   # Create simple test script
   python3 verify_ros2_calibration.py
   ```

3. **Baseline Check**: From calibration files
   ```bash
   # Extract Tx from right camera projection matrix
   grep -A 3 "projection_matrix" calibration_ros2/right.yaml
   # Tx should be approximately -0.06 (60mm baseline in meters)
   ```

4. **Visual Inspection**: Rectified images should be aligned
   - Same points appear at same Y-coordinate
   - Horizontal lines pass through corresponding points

---

## Alternative: April Tags Calibration

If chessboard calibration is unsuccessful, April Tags provide more robust detection.

### Generate April Tags

1. Use April Tag generator: https://github.com/AprilRobotics/apriltag-imgs
2. Print 6x6 grid of tags (family: 36h11)
3. Measure tag size and spacing accurately

### Calibrate with April Tags

```bash
# Different command for April Tags
ros2 run camera_calibration cameracalibrator \
  --pattern apriltag \
  --size 6x6 \
  --square 0.025 \
  --approximate 0.1 \
  right:=/stereo/right/image_raw \
  left:=/stereo/left/image_raw
```

---

## Next Steps

After successful calibration:

1. ✅ **Test depth maps** with stereo_image_proc
2. ✅ **Integrate with robot framework** (use calibrated parameters)
3. ✅ **Object detection** with 3D pose estimation
4. ✅ **Visual servoing** for robot arm control
5. 📝 **Add IMU later** if hardware becomes available

---

## References

- ROS2 camera_calibration: http://wiki.ros.org/camera_calibration
- stereo_image_proc: http://wiki.ros.org/stereo_image_proc
- OpenCV camera calibration: https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html
- April Tags: https://github.com/AprilRobotics/apriltag

---

## Files Created During Calibration

```
/home/jay/projects/stereo_camera/
├── calibration_ros2/          # ROS2 calibration results
│   ├── left.yaml              # Left camera parameters
│   ├── right.yaml             # Right camera parameters
│   └── ost.yaml               # Combined stereo calibration
├── calibration_ros2_*.tar.gz  # Backup archives (timestamped)
└── launch_stereo_proc.sh      # Launch stereo processing
```

---

**Last Updated**: 2025-10-16
**Status**: Ready for use after package installation
