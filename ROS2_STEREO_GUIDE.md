# ROS2 Stereo Camera Guide

คู่มือการใช้งาน ROS2 กับ Stereo Camera บน Jetson Orin Nano

## สิ่งที่เตรียมไว้แล้ว

✅ jetson-containers - Official NVIDIA containers สำหรับ Jetson  
✅ ROS2 Humble image - รองรับ Argus camera  
✅ Stereo Camera Node - Python node สำหรับ publish stereo images  
✅ Helper scripts - สคริปต์เริ่มต้น container

## ไฟล์สำคัญ

- `start_ros2_stereo.sh` - สคริปต์เริ่มต้น ROS2 container
- `gstreamer_camera_node.py` - ROS2 node สำหรับ stereo camera
- `/home/jay/jetson-containers/` - jetson-containers repository

---

## ขั้นตอนการใช้งาน

### 1. เริ่มต้น ROS2 Container

```bash
cd /home/jay/projects/stereo_camera
./start_ros2_stereo.sh
```

Container จะ:
- Mount workspace ที่ `/workspace`
- รองรับ Argus camera ผ่าน `/tmp/argus_socket`
- เปิด device `/dev/video*` และ `/dev/i2c-*` ทั้งหมด
- ตั้งค่า ROS_DOMAIN_ID=0

### 2. ทดสอบกล้องใน Container

```bash
# ทดสอบกล้องซ้าย (sensor_id=0)
gst-launch-1.0 nvarguscamerasrc sensor-id=0 num-buffers=30 ! \
  'video/x-raw(memory:NVMM),width=1280,height=720,framerate=30/1' ! \
  nvvidconv ! xvimagesink

# ทดสอบกล้องขวา (sensor_id=1)
gst-launch-1.0 nvarguscamerasrc sensor-id=1 num-buffers=30 ! \
  'video/x-raw(memory:NVMM),width=1280,height=720,framerate=30/1' ! \
  nvvidconv ! xvimagesink
```

### 3. ติดตั้ง Dependencies (ใน Container)

```bash
# Install cv_bridge และ dependencies
apt-get update
apt-get install -y python3-opencv ros-humble-cv-bridge

# หรือถ้ามี pip
pip3 install opencv-python
```

### 4. รัน Stereo Camera Node

```bash
# ใน container
cd /workspace

# รัน node โดยตรง
python3 gstreamer_camera_node.py

# หรือรันผ่าน ros2 run (ต้อง setup package ก่อน)
ros2 run stereo_camera stereo_camera_node
```

### 5. ตรวจสอบ Topics

เปิด terminal ใหม่และเข้า container:

```bash
docker exec -it ros2_stereo_camera bash
```

ตรวจสอบ topics:

```bash
# ดู topics ทั้งหมด
ros2 topic list

# ควรเห็น:
# /stereo/left/image_raw
# /stereo/right/image_raw
# /stereo/left/camera_info
# /stereo/right/camera_info

# ดูข้อมูล topic
ros2 topic info /stereo/left/image_raw

# ดู message rate
ros2 topic hz /stereo/left/image_raw

# Echo topic (ระวัง: ข้อมูลเยอะ!)
ros2 topic echo /stereo/left/camera_info
```

### 6. Visualize ด้วย RViz2

```bash
# ติดตั้ง RViz2 (ถ้ายังไม่มี)
apt-get install -y ros-humble-rviz2

# รัน RViz2
rviz2
```

ใน RViz2:
1. เปลี่ยน Fixed Frame เป็น `left_camera_optical_frame`
2. Add → By topic → `/stereo/left/image_raw` → Image
3. Add → By topic → `/stereo/right/image_raw` → Image

---

## การปรับแต่ง Camera Node

แก้ไขไฟล์ `gstreamer_camera_node.py`:

```python
# ปรับ parameters
self.declare_parameter('left_sensor_id', 0)    # เปลี่ยน sensor ID
self.declare_parameter('right_sensor_id', 1)
self.declare_parameter('width', 1280)          # เปลี่ยน resolution
self.declare_parameter('height', 720)
self.declare_parameter('framerate', 30)        # เปลี่ยน framerate
```

หรือส่ง parameters ตอน runtime:

```bash
python3 gstreamer_camera_node.py --ros-args \
  -p width:=640 -p height:=480 -p framerate:=60
```

---

## Troubleshooting

### ❌ ปัญหา: "Failed to open camera"

**สาเหตุ**: argus daemon ไม่ทำงาน หรือ socket ไม่ถูก mount

**วิธีแก้**:
```bash
# ตรวจสอบ argus socket บน host
ls -la /tmp/argus_socket

# ตรวจสอบใน container
docker exec ros2_stereo_camera ls -la /tmp/argus_socket

# Restart container
docker stop ros2_stereo_camera
docker rm ros2_stereo_camera
./start_ros2_stereo.sh
```

### ❌ ปัญหา: "No module named 'cv_bridge'"

**วิธีแก้**:
```bash
apt-get update
apt-get install -y ros-humble-cv-bridge python3-opencv
```

### ❌ ปัญหา: "nvarguscamerasrc not found"

**สาเหตุ**: ใช้ image ที่ไม่รองรับ Argus

**วิธีแก้**: ใช้ official jetson-containers image:
```bash
# แก้ไขใน start_ros2_stereo.sh
IMAGE="dustynv/ros:humble-ros-base-l4t-r36.2.0"
```

### ❌ ปัญหา: ภาพ lag หรือ framerate ต่ำ

**วิธีแก้**:
1. ลด resolution (1280x720 → 640x480)
2. ลด framerate (30 → 15)
3. ใช้ `--shm-size=8g` (มีอยู่แล้วใน script)

---

## ขั้นตอนถัดไป

### Phase 1: ✅ ROS2 Setup (เสร็จแล้ว)
- ✅ ติดตั้ง jetson-containers
- ✅ สร้าง ROS2 Humble container
- ✅ สร้าง Stereo Camera Node
- ⏳ ทดสอบ camera publishing

### Phase 2: Kalibr Calibration
- ติดตั้ง Kalibr
- สร้าง AprilTag calibration target
- จับข้อมูล calibration (rosbag)
- รัน Kalibr multi-camera + IMU calibration
- อัพเดท CameraInfo messages

### Phase 3: Stereo Perception
- Setup stereo_image_proc
- Generate depth maps
- Point cloud generation
- Depth visualization

### Phase 4: Object Detection
- Train/adapt YOLO model
- 3D pose estimation
- Integration กับ perception pipeline

### Phase 5: Robot Integration
- Hand-eye calibration
- Motion planning (MoveIt2)
- Grasping control

---

## เอกสารอ้างอิง

- [jetson-containers GitHub](https://github.com/dusty-nv/jetson-containers)
- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [Kalibr Documentation](https://github.com/ethz-asl/kalibr)
- [NVIDIA Argus Camera API](https://docs.nvidia.com/jetson/l4t-multimedia/group__LibargusAPI.html)

---

**สร้างเมื่อ**: 2025-10-16  
**Status**: Phase 1 - ROS2 Setup & Camera Node
