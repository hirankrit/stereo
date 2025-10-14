# Stereo Camera IMX219 on Jetson Orin Nano

A comprehensive project for setting up and using dual IMX219 stereo cameras on NVIDIA Jetson Orin Nano for computer vision applications.

![Jetson Orin Nano](https://img.shields.io/badge/Jetson-Orin%20Nano-76B900?logo=nvidia&logoColor=white)
![JetPack 6.2](https://img.shields.io/badge/JetPack-6.2-76B900)
![Python 3](https://img.shields.io/badge/Python-3.x-3776AB?logo=python&logoColor=white)
![OpenCV](https://img.shields.io/badge/OpenCV-4.x-5C3EE8?logo=opencv&logoColor=white)

## 📋 Table of Contents

- [Hardware Requirements](#hardware-requirements)
- [System Information](#system-information)
- [Features](#features)
- [Installation](#installation)
- [Usage](#usage)
- [Project Files](#project-files)
- [Troubleshooting](#troubleshooting)
- [Next Steps](#next-steps)
- [Resources](#resources)

## 🔧 Hardware Requirements

- **Board**: NVIDIA Jetson Orin Nano Engineering Reference Developer Kit Super
- **Camera**: IMX219-83 Stereo Camera (Dual IMX219 sensors)
- **Connection**: CSI-2 interface

## 💻 System Information

- **OS**: Ubuntu (Linux 5.15.148-tegra)
- **JetPack**: 6.2 (R36.4.4)
- **Kernel**: 5.15.148-tegra
- **Camera Driver**: tegracam sensor driver imx219_v2.0.6

## ✨ Features

- ✅ Dual IMX219 camera support (1920x1080 @ 30fps)
- ✅ Real-time stereo camera viewer
- ✅ Device tree overlay configuration
- ✅ GStreamer-based camera pipeline
- ✅ OpenCV integration
- 🔄 Ready for stereo calibration
- 🔄 Ready for depth map generation

## 📦 Installation

### Step 1: Clone the Repository

```bash
git clone https://github.com/hirankrit/stereo.git
cd stereo
```

### Step 2: Enable IMX219 Stereo Camera

Run the installation script to configure device tree overlay:

```bash
chmod +x enable_imx219_stereo.sh
sudo ./enable_imx219_stereo.sh
```

This script will:
1. Backup your current boot configuration
2. Detect the correct DTB file
3. Merge the IMX219 stereo camera overlay
4. Update boot configuration

### Step 3: Reboot

After the script completes, reboot your Jetson:

```bash
sudo reboot
```

### Step 4: Verify Camera Detection

After reboot, verify that both cameras are detected:

```bash
# Check video devices
ls /dev/video*

# Should show /dev/video0 and /dev/video1

# Check I2C devices
i2cdetect -y -r 9   # Left camera (0x10)
i2cdetect -y -r 10  # Right camera (0x10)
```

### Step 5: Install Python Dependencies

```bash
sudo apt-get update
sudo apt-get install python3-opencv python3-numpy
```

## 🚀 Usage

### Test Individual Cameras

Test left camera (sensor-id=0):
```bash
gst-launch-1.0 nvarguscamerasrc sensor-id=0 num-buffers=30 ! \
  'video/x-raw(memory:NVMM),width=1920,height=1080,framerate=30/1' ! \
  nvvidconv ! xvimagesink
```

Test right camera (sensor-id=1):
```bash
gst-launch-1.0 nvarguscamerasrc sensor-id=1 num-buffers=30 ! \
  'video/x-raw(memory:NVMM),width=1920,height=1080,framerate=30/1' ! \
  nvvidconv ! xvimagesink
```

### Run Stereo Camera Viewer

Display both cameras side-by-side in real-time:

```bash
python3 stereo_view.py
```

**Controls:**
- Press `q` to quit
- Press `s` to save snapshot

### Run Camera Test Script

```bash
python3 test_stereo_camera.py
```

This script will:
- Open both stereo cameras
- Display real-time video feed
- Allow you to save calibration images
- Show FPS counter

## 📁 Project Files

| File | Description |
|------|-------------|
| `README.md` | This file - project documentation |
| `claude.md` | Detailed setup guide and troubleshooting notes |
| `enable_imx219_stereo.sh` | Device tree overlay installation script |
| `test_stereo_camera.py` | Python script for testing stereo cameras |
| `stereo_view.py` | Real-time stereo camera viewer application |

## 🔍 Troubleshooting

### Issue: "No cameras available" Error

**Cause**: Device tree overlay not loaded

**Solution**:
1. Verify the overlay was merged correctly:
```bash
find /proc/device-tree -name "*imx219*"
```

2. Check boot configuration:
```bash
cat /boot/extlinux/extlinux.conf
```

3. Re-run the installation script:
```bash
sudo ./enable_imx219_stereo.sh
sudo reboot
```

### Issue: Only One Camera Working

**Cause**: Camera not properly connected or I2C bus issue

**Solution**:
1. Check physical connections
2. Verify I2C detection:
```bash
i2cdetect -y -r 9   # Bus 9: Left camera
i2cdetect -y -r 10  # Bus 10: Right camera
```

3. Check kernel logs:
```bash
dmesg | grep -i imx219
```

### Issue: "Failed to create CaptureSession"

**Cause**: Camera initialization timing issue

**Solution**: Try running the test again. The first run may fail, but subsequent runs should work.

## 🎯 Next Steps

### 1. Stereo Camera Calibration

Calibrate your stereo camera setup for accurate depth estimation:

```bash
# Capture calibration images (chessboard pattern)
python3 test_stereo_camera.py
# Press 's' to save multiple images from different angles

# Run calibration script (to be implemented)
# python3 calibrate_stereo.py
```

### 2. Depth Map Generation

Create depth maps from stereo images:
- Stereo matching algorithms
- Block matching
- Semi-global block matching (SGBM)

### 3. Computer Vision Applications

- 3D reconstruction
- Object detection with distance measurement
- Obstacle avoidance
- SLAM (Simultaneous Localization and Mapping)
- Visual odometry

### 4. Performance Optimization

- CUDA acceleration
- TensorRT integration
- Custom GStreamer pipelines
- Hardware-accelerated stereo matching

## 📚 Resources

### Official Documentation

- [NVIDIA Jetson Linux Developer Guide](https://docs.nvidia.com/jetson/l4t/)
- [GStreamer on Jetson](https://developer.nvidia.com/embedded/learn/tutorials/first-picture-csi-usb-camera)
- [IMX219 Camera Module Datasheet](https://www.sony-semicon.com/files/62/flyer/p-11_e.pdf)

### Useful Commands

```bash
# List video devices
v4l2-ctl -d /dev/media0 --list-devices
ls /dev/video*

# Test camera with GStreamer
gst-launch-1.0 nvarguscamerasrc sensor-id=0 ! nvvidconv ! xvimagesink

# Check kernel messages
dmesg | grep -i camera
dmesg | grep -i imx219

# Check I2C buses
i2cdetect -y -r 9
i2cdetect -y -r 10

# View device tree
cat /proc/device-tree/model
find /proc/device-tree -name "*imx219*"
```

## 📝 Camera Specifications

| Specification | Value |
|---------------|-------|
| Sensor | Sony IMX219 |
| Resolution | 3280 x 2464 (8MP) |
| Max FPS | 21 fps @ full resolution |
| Recommended | 1920x1080 @ 30fps |
| Interface | CSI-2 (2-lane) |
| Pixel Size | 1.12 μm |
| Optical Format | 1/4" |

## 🤝 Contributing

Contributions are welcome! Please feel free to submit pull requests or open issues for bugs and feature requests.

## 📄 License

This project is provided as-is for educational and research purposes.

## 👤 Author

**hirankrit**
- GitHub: [@hirankrit](https://github.com/hirankrit)

## 🙏 Acknowledgments

- NVIDIA Jetson Community
- OpenCV Community
- Sony Semiconductor Solutions

---

**Status**: ✅ Both stereo cameras working successfully! Ready for calibration and stereo vision applications.

**Last Updated**: October 8, 2025
