# โปรเจค: Stereo Vision + IMU สำหรับแขนหุ่นยนต์เกษตร (Jetson Orin Nano)

## เป้าหมายโปรเจค 🎯🤖🌾

**วัตถุประสงค์**: พัฒนาระบบ stereo vision + IMU สำหรับแขนหุ่นยนต์เก็บ/คัดพืชผลทางการเกษตร

**ระบบ**:
- Stereo Camera (IMX219) + IMU (ICM20948) → 3D perception + motion sensing
- ROS2 ecosystem → robot integration
- Calibration: Kalibr → camera-IMU calibration
- Applications: Depth estimation, object detection, pose estimation, motion planning

**Use Case**: Agricultural robot arm for harvesting/sorting crops

---

## ข้อมูลระบบ
- **บอร์ด**: NVIDIA Jetson Orin Nano Engineering Reference Developer Kit Super
- **OS**: Ubuntu (Linux 5.15.148-tegra)
- **JetPack**: 6.2 (R36.4.4)
- **กล้อง**: IMX219-83 Stereo Camera (Dual IMX219 sensors)

### ข้อมูลกล้อง (Hardware Specifications)

**โมดูลกล้อง**: โมดูลกล้องสองตาแบบคู่ IMX219, 8MP

**เซ็นเซอร์**:
- Model: IMX219
- Resolution: 8 ล้านพิกเซล (3280 × 2464 ต่อกล้อง)
- Sensor Size: 1/4 นิ้ว
- Pixel Size: 1.12 µm

**เลนส์**:
- Focal Length: 1.6mm
- Aperture: F2.4
- Field of View: 83° (Diagonal), 73° (Horizontal), 50° (Vertical)
- Distortion: ~1%

**การจัดวาง**:
- **Baseline (ระยะห่างระหว่างกล้อง)**: **60mm** (6 cm) - วัดจากกลางเลนส์ถึงกลางเลนส์
- Mount: บน PCB เดียวกัน (แข็งแรง, ควรจะ parallel)
- Layout: กล้องซ้าย-ขวาเรียงแนวนอน

**IMU (Inertial Measurement Unit)**:
- Model: ICM20948 (9-axis)
- Accelerometer: ±2/4/8/16g, 16-bit
- Gyroscope: ±250/500/1000/2000°/s, 16-bit
- Magnetometer: ±4900μT, 16-bit

**ขนาดโมดูล**: 24 × 85 mm

**การรองรับ**:
- Jetson Nano Developer Kit (B01)
- Jetson Orin Nano
- Raspberry Pi CM3/CM3+/CM4

**แอปพลิเคชันที่แนะนำ**:
- การมองเห็นเชิงลึก (Depth Perception)
- การมองเห็นแบบสเตอริโอ (Stereo Vision)
- Visual SLAM
- 3D Reconstruction

## สถานะปัจจุบัน

### 🔄 วันที่ 2025-10-16 (Late Evening): Phase 2 - IMU Investigation & Calibration Setup

#### สรุปการทำงานวันนี้:

**Phase 2 Attempt: IMU Setup** ❌→✅ (เปลี่ยนแผน)
- ❌ **ไม่พบ ICM20948 IMU** หลังตรวจสอบทั้งระบบ
- ✅ สร้างเอกสาร `IMU_INVESTIGATION.md` - สรุปการตรวจสอบโดยละเอียด
- ✅ ตัดสินใจ: **ดำเนินการต่อโดยไม่มี IMU ก่อน** (Camera-only calibration)

**การตรวจสอบ IMU**:
- Scanned I2C buses: 0, 1, 2, 4, 5, 7, 9, 10, 11 - ไม่พบ ICM20948
- Devices ที่พบ:
  - Bus 1, 0x25: `fusb301` (USB Type-C controller)
  - Bus 1, 0x40: `ina3221` (Power monitor)
  - Bus 2, 9: IMX219 cameras (working ✓)
- Checked SPI buses: มี 4 SPI devices แต่ไม่มี ICM20948 driver
- Device tree: ไม่มี IMU nodes
- IIO devices: ไม่มี

**สาเหตุที่เป็นไปได้**:
1. IMU ไม่มีใน hardware module จริง (แม้ว่าจะมีในสเปค)
2. IMU อยู่บน SPI แต่ต้อง enable ผ่าน device tree
3. ต้องซื้อ/ติดตั้ง IMU module แยกต่างหาก

**แผนใหม่ - ROS2 Camera Calibration**:
- ✅ เปลี่ยนจาก Kalibr (camera+IMU) → **ROS2 camera_calibration** (camera-only)
- ✅ สร้างสคริปต์ติดตั้ง: `install_ros2_calibration.sh`
- Packages ที่จะติดตั้ง:
  - `ros-humble-camera-calibration` - Stereo calibration
  - `ros-humble-image-pipeline` - Stereo processing
  - `ros-humble-rqt-image-view` - Image viewer
- จะใช้ chessboard pattern ที่มี (6x6, 25mm) หรือสร้าง April tag

**Tools ที่สร้าง**:
- ✅ `scan_imu.py` - I2C scanner สำหรับหา ICM20948
- ✅ `scan_all_i2c.py` - Full I2C bus scanner
- ✅ `IMU_INVESTIGATION.md` - เอกสารสรุปการตรวจสอบ IMU
- ✅ `install_ros2_calibration.sh` - ติดตั้ง ROS2 calibration tools

**งานถัดไป**:
- [ ] รัน `install_ros2_calibration.sh` เพื่อติดตั้ง calibration tools
- [ ] ทดสอบ ROS2 camera_calibration กับ stereo camera
- [ ] สร้าง calibration target (chessboard หรือ April tag)
- [ ] จับภาพและทำ calibration
- [ ] ตรวจสอบคุณภาพ calibration

---

### ✅ วันที่ 2025-10-16 (Evening): ROS2 + Visualization Complete! 🎉📸

#### งานที่เสร็จทั้งหมด:

**1. ติดตั้ง ROS2 Humble บน Host** ✅
- **เหตุผล**: Container image มีปัญหา (NVIDIA libraries เป็น empty files)
- **วิธีแก้**: ติดตั้ง ROS2 Humble โดยตรงบน Jetson Orin Nano
- ติดตั้ง ROS2 Humble base + dependencies
- ติดตั้ง cv_bridge, sensor_msgs, std_msgs
- Initialize rosdep สำเร็จ
- **ผลลัพธ์**: ROS2 ทำงานได้สมบูรณ์บน host

**2. Stereo Camera Node ทำงานสมบูรณ์** ✅
- ไฟล์: `gstreamer_camera_node.py`
- เป็น **StereoCameraNode** สำหรับ stereo camera
- Features:
  - รองรับ stereo camera (left + right) พร้อมกัน
  - ใช้ nvarguscamerasrc สำหรับ Argus camera
  - Publish ไปที่ ROS2 topics:
    - `/stereo/left/image_raw` - Left camera images
    - `/stereo/right/image_raw` - Right camera images
    - `/stereo/left/camera_info` - Left camera info
    - `/stereo/right/camera_info` - Right camera info
  - รองรับ parameters: resolution (1280x720), framerate (30fps), sensor_id
  - Synchronized timestamps สำหรับ stereo pairs
  - **แก้ไข**: เพิ่ม delay 2 วินาทีระหว่างการเปิดกล้อง (แก้ปัญหา resource conflict)

**3. ทดสอบ ROS2 Topics สำเร็จ** ✅
- **Frame Rate**: ~20 fps (target: 30 fps)
- **Resolution**: 1280x720 BGR8
- **Performance**: ไม่มี frame drops, stable streaming
- **Published**: 4800+ stereo frame pairs
- **Topics verified**:
  ```
  /stereo/left/image_raw       - sensor_msgs/Image
  /stereo/right/image_raw      - sensor_msgs/Image
  /stereo/left/camera_info     - sensor_msgs/CameraInfo
  /stereo/right/camera_info    - sensor_msgs/CameraInfo
  ```

**4. ติดตั้ง Visualization Tools** ✅
- **RViz2**: Advanced 3D visualization tool
- **rqt_image_view**: Simple image viewer (แนะนำใช้)
- ทดสอบการติดตั้งสำเร็จ

**5. สร้าง Helper Scripts** ✅
- `run_stereo_node.sh` - เริ่ม Stereo Camera Node
  - หยุด node เก่าอัตโนมัติ
  - รอกล้องถูกปล่อย (2 วินาที)
  - รัน node ใหม่
- `view_stereo_cameras.sh` - เปิด rqt_image_view
- `start_stereo_system.sh` - เปิดทั้งระบบพร้อมกัน (node + viewer)
  - เริ่ม camera node ใน background
  - รอ initialization
  - เช็ค topics
  - เปิด viewer

**6. เอกสารครบถ้วน** ✅
- `ROS2_STEREO_GUIDE.md` - คู่มือ ROS2 stereo camera
- `VIEWER_GUIDE.md` - คู่มือการใช้งาน visualization tools
  - rqt_image_view usage
  - RViz2 setup
  - Troubleshooting
  - Performance tips

#### สิ่งที่เตรียมพร้อมแล้ว:

**Software Stack**:
- ✅ ROS2 Humble (ติดตั้งบน host)
- ✅ Argus camera support (nvarguscamerasrc)
- ✅ Stereo Camera Node (Python, ทำงานสมบูรณ์)
- ✅ cv_bridge และ ROS2 dependencies
- ✅ RViz2 และ rqt_image_view
- ✅ Helper scripts และคู่มือ

**การทำงาน**:
- ✅ Node publish stereo images @ ~20 fps
- ✅ Topics ทั้งหมดพร้อมใช้งาน
- ✅ Visualization tools พร้อมดูภาพ real-time
- ✅ System มั่นคง ไม่มี frame drops

**บทเรียน**:
- ✅ **Container approach ไม่เหมาะสม**: jetson-containers image มีปัญหา NVIDIA libraries
- ✅ **Host installation ทำงานได้ดีกว่า**: ROS2 บน host ใช้งานได้เต็มประสิทธิภาพ
- ✅ **Camera opening delay จำเป็น**: ต้อง delay 2 วินาทีระหว่างเปิดกล้อง 2 ตัว
- ✅ **rqt_image_view เบากว่า RViz2**: เหมาะสำหรับ simple viewing

---

### 🔄 วันที่ 2025-10-16 (Morning): การเปลี่ยนแปลงทิศทาง - ROS + Kalibr Approach

#### สรุปผลการทดลอง OpenCV Stereo Calibration (รอบที่ 1-5)

**ผลการทดสอบ**:

| Round | Description | Images | Intensity | Individual RMS | Stereo RMS | Baseline | Status |
|-------|-------------|--------|-----------|----------------|------------|----------|--------|
| 1-2 | Initial attempts | 7-56 | N/A | Poor | 25-70px | 1.9m-7.1m | ❌ Failed |
| 3 | Dark images | 82 | 47-53 | 0.29-0.31 | 19.43px | 1219mm | ❌ Failed |
| 4 | Bright images | 40 | 114-116 | 0.30-0.44 | 116.33px | 2744mm | ❌ Failed |
| 5 | **Shared intrinsics** | 40 | 114-116 | 0.30-0.44 | **52.01px** | **156mm** | ⚠️ Improved |

**หมายเหตุ**: ค่า baseline ที่ถูกต้อง = **60mm** (วัดจากฮาร์ดแวร์)

**การวิเคราะห์**:
- ✅ **Individual camera calibration**: ดีเยี่ยม (RMS < 0.5 pixels)
- ✅ **แสงสว่าง**: แก้ไขสำเร็จ (mean intensity เพิ่มจาก 47-53 → 114-116)
- ✅ **Shared intrinsics**: ช่วยได้มาก (Stereo RMS ลดลง 55%, Baseline ดีขึ้น 17 เท่า)
- ❌ **Stereo calibration**: ยังล้มเหลว (Baseline error 161%, Stereo RMS 52 pixels, Rotation ~16°)

**ปัญหาหลัก**:
1. **OpenCV stereo calibration algorithm ไม่เหมาะสม** - สมมติว่ากล้อง parallel แต่อาจมีมุม ~16°
2. **Valid pairs ไม่พอ** - 31/40 (78%), กล้องขวาตรวจจับไม่ได้หลายภาพ
3. **Hardware alignment** - อาจมีมุมเล็กน้อยที่ OpenCV ไม่สามารถจัดการได้

**บทเรียนจาก OpenCV approach**:
- ✅ AUTO-CAPTURE mode ทำงานดี
- ✅ Pattern detection (6x6 inner corners, 25mm) ใช้ได้ดี
- ✅ Image quality มีผลต่อผลลัพธ์
- ❌ OpenCV `cv2.stereoCalibrate()` ไม่เหมาะกับ hardware นี้
- ❌ ต้องการ calibration tool ที่ดีกว่า

---

#### 🚀 ทิศทางใหม่: ROS + Kalibr + IMU Integration

**เหตุผลในการเปลี่ยนแปลง**:
1. **เป้าหมายสุดท้าย**: แขนหุ่นยนต์เกษตร → ต้องใช้ ROS ecosystem
2. **IMU onboard**: ICM20948 (9-axis) → ควรใช้ให้เกิดประโยชน์
3. **Kalibr > OpenCV**: Gold standard สำหรับ multi-camera + IMU calibration
4. **ความยืดหยุ่น**: ไม่บังคับให้กล้อง parallel, หาความสัมพันธ์ได้แม่นยำกว่า

**Roadmap ใหม่**:

**Phase 1: ROS2 Setup** ✅ **COMPLETED 2025-10-16 Evening**
- [x] ~~ติดตั้ง jetson-containers~~ → เปลี่ยนเป็น install บน host แทน
- [x] ติดตั้ง ROS2 Humble บน host (native installation)
- [x] ติดตั้ง cv_bridge และ dependencies
- [x] สร้าง Stereo Camera Node: `gstreamer_camera_node.py`
- [x] ทดสอบ camera publishing ใน ROS2 - **สำเร็จ @ 20 fps**
- [x] ติดตั้ง RViz2 และ rqt_image_view
- [x] สร้าง helper scripts: `run_stereo_node.sh`, `start_stereo_system.sh`, `view_stereo_cameras.sh`
- [x] เอกสาร: `ROS2_STEREO_GUIDE.md`, `VIEWER_GUIDE.md`
- [x] Verify ROS2 topics - **ทั้งหมดทำงานสมบูรณ์**
- [x] ~~Setup IMU driver~~ → **IMU ไม่พบ** (ดู IMU_INVESTIGATION.md)
- [x] ตัดสินใจ: **ดำเนินการต่อแบบ camera-only**

**Phase 2: Camera Calibration** 🔄 **IN PROGRESS 2025-10-16 Late Evening**
- [x] ~~ติดตั้ง Kalibr~~ → เปลี่ยนเป็น **ROS2 camera_calibration** (camera-only)
- [x] สร้าง `install_ros2_calibration.sh` - ติดตั้ง calibration tools
- [ ] รันสคริปต์ติดตั้ง calibration packages
- [ ] สร้าง calibration target (ใช้ chessboard 6x6 ที่มี หรือ April tags)
- [ ] จับข้อมูล calibration (rosbag หรือ live camera)
- [ ] รัน ROS2 camera_calibration สำหรับ stereo
- [ ] Verify calibration results

**Phase 3: Stereo Perception** (Week 5-6)
- [ ] Setup stereo_image_proc
- [ ] Generate depth maps
- [ ] Point cloud generation
- [ ] Depth visualization

**Phase 4: Object Detection** (Week 7-8)
- [ ] Train/adapt detection model (YOLO for crops)
- [ ] 3D pose estimation
- [ ] Integration with perception pipeline

**Phase 5: Robot Integration** (Week 9-12)
- [ ] Hand-eye calibration
- [ ] Motion planning (MoveIt2)
- [ ] Grasping control
- [ ] Full system integration

**ข้อดีของ ROS2 Camera Calibration approach** (แก้ไขจาก Kalibr):
- ✅ ROS ecosystem → integration กับ robot arm ง่าย
- ✅ camera_calibration → เหมาะสำหรับ stereo camera
- ✅ Native ARM64 support (ไม่ต้อง Docker)
- ✅ Community support ใหญ่
- ✅ stereo_image_proc → depth maps ได้ทันที
- ✅ เหมาะสำหรับ agricultural robotics
- 📝 เมื่อมี IMU ภายหลังสามารถเพิ่ม sensor fusion ได้

**Tools ที่ใช้** (อัพเดทแล้ว):
- **ROS2 Humble**: Robot framework ✅
- **camera_calibration**: Stereo camera calibration (แทน Kalibr)
- **stereo_image_proc**: Stereo processing & depth maps
- **image_pipeline**: Complete image processing
- **MoveIt2**: Motion planning (ในอนาคต)
- **YOLO**: Object detection (ในอนาคต)
- **PCL**: Point cloud processing (ในอนาคต)

---

### วันที่ 2025-10-15 (Session 7: Stereo Calibration Issues - Deep Investigation 🔍❌)

#### ✅ งานที่เสร็จแล้ว:

0. **ยืนยันข้อมูล Hardware Specifications** ✅
   - ตรวจสอบรายละเอียดกล้องจากผู้ผลิต
   - ดูรูปภาพโมดูลกล้อง
   - **ยืนยัน Baseline จริง: 60mm** (วัดจากกลางเลนส์ถึงกลางเลนส์)
   - กล้องทั้งสองติดบน PCB เดียวกัน → mount แข็งแรง, parallel
   - Sensor: IMX219, 8MP, focal length 1.6mm, FOV 83°
   - มี IMU (ICM20948) 9-axis onboard
   - ขนาดโมดูล: 24 × 85 mm

1. **จับภาพ Calibration Images ใหม่** ✅
   - ปรับ gain: ลอง 8 → กลับเป็น 10 (ภาพสว่างที่สุด)
   - จับภาพสำเร็จ: **80 คู่!** (เกินเป้าหมาย 20-30 คู่)
   - AUTO-CAPTURE mode ทำงานสมบูรณ์
   - Detection สำเร็จทุกมุม - ผู้ใช้บอก "สุดยอดมุมไหนก็ detect ได้หมดเลย"
   - ใช้เวลา: ~12 นาที (80 ภาพ @ 3 วิ/ภาพ)

2. **Calibration Round 3** (82 valid pairs, 1280x720, gain=10)
   - ✅ Left RMS: **0.29 pixels** (ดีเยี่ยม!)
   - ✅ Right RMS: **0.31 pixels** (ดีเยี่ยม!)
   - ❌ Stereo RMS: **19.43 pixels** (ยังสูงมากเหมือนเดิม!)
   - ❌ Baseline: **1219 mm** (122 cm) - **ผิดพลาดมาก!** (ควร ~60-70mm)
   - ใช้เวลา calibration: ~15 นาที (timeout หลายครั้ง)

3. **ตรวจสอบ verify_calibration.py** ⚠️
   - **ปัญหา**: ภาพยังเป็นเส้นๆ อีกครั้ง (ไม่มีภาพจริง)
   - **แม้จะแก้ resolution mismatch แล้ว**!

4. **วิเคราะห์ปัญหา Stereo Calibration** ✅
   - **ตรวจสอบภาพ**: สร้าง `check_stereo_pair.jpg`
   - ✅ **ภาพทั้งสองเป็นฉากเดียวกัน** - stereo pair ถูกต้อง
   - ✅ **ภาพเยื้องกัน ~1 นิ้ว (2.5 cm)** - เป็นปกติสำหรับ 6-7cm baseline
   - ❌ **ภาพมืดมาก**: mean 47-53 (จาก 255), max 109-255
   - ❌ **Focal length ต่างกันมาก**: Left = 2705px, Right = 4749px
   - ❌ **Rotation มาก**: 37.6° (ควรใกล้ 0°)
   - ❌ **Translation ผิดพลาด**: [-647, -156, 1022] mm แทน [±60, ~0, ~0]

5. **ทดลองแก้ไข Calibration Flags** ⚠️
   - **Attempt 1**: ใช้ `CALIB_USE_INTRINSIC_GUESS` แทน `CALIB_FIX_INTRINSIC`
     - ผลลัพธ์: Timeout (>15 นาที)

   - **Attempt 2**: ใช้ subset 30 ภาพ (ลดจาก 82)
     - ✅ Left RMS: 0.35, Right RMS: 0.31
     - ❌ Stereo RMS: **26.54** (แย่ลง!)
     - ❌ Baseline: **1887 mm** (1.9m - แย่ลงอีก!)
     - Focal length: Left = 10018, Right = 5163

   - **Attempt 3**: Averaged focal length + `CALIB_FIX_INTRINSIC`
     - ผลลัพธ์: Timeout (>10 นาที), calibration ไม่เสร็จ

#### ❌ ปัญหาหลักที่พบ:

**1. Stereo Calibration ล้มเหลวอย่างสิ้นเชิง**
   - Stereo RMS: 19-27 pixels (ควร < 1.0)
   - Baseline: 1.2-1.9 เมตร (ควร 6-7 cm)
   - **Error มากกว่า 2000%!**

**2. Focal Length ไม่สอดคล้องกัน**
   - กล้องทั้งสองเป็น IMX219 เดียวกัน แต่ focal length ต่างกัน 1.5-2 เท่า
   - Left: 2705-10018 px
   - Right: 4749-5163 px
   - **แสดงว่า individual calibration ผิดพลาด**

**3. Rotation และ Translation ผิดปกติ**
   - Rotation: 37.6° (ควรใกล้ 0° สำหรับกล้อง parallel)
   - Translation: [-647, -156, 1022] mm (ควร [±60, ~0, ~0])
   - **แสดงว่า stereo calibration algorithm คำนวณความสัมพันธ์ผิดพลาด**

**4. Rectification Maps ใช้งานไม่ได้**
   - verify_calibration.py แสดงเฉพาะเส้น epipolar lines
   - ไม่แสดงภาพจริง (สีน้ำตาล/ดำ)
   - **Rectification maps ถูกสร้างจาก baseline ที่ผิด → ใช้งานไม่ได้**

**5. ภาพมืดเกินไป**
   - Mean intensity: 47-53 (จาก 255)
   - Max intensity: 109 (left), 255 (right)
   - **อาจส่งผลต่อ corner detection accuracy**

#### 🔍 การวิเคราะห์สาเหตุ:

**สาเหตุที่เป็นไปได้**:

1. **กล้องไม่ Parallel** 🎯 ~~(สาเหตุหลัก)~~ → **แต่กล้องบน PCB เดียวกัน ควรจะ parallel!**
   - ✅ **Hardware mounting**: กล้องทั้งสองติดบน PCB เดียวกัน
   - ✅ **Baseline จริง: 60mm** (วัดแล้ว)
   - ❌ แต่ calibration ได้ rotation 37° และ baseline 1219mm
   - 🤔 **ปัญหาอาจไม่ใช่ hardware แต่เป็น software/algorithm!**

2. **ระยะ Chessboard ไม่เหมาะสม**
   - ระยะ 41-43 cm อาจจะไม่เหมาะสมสำหรับ baseline 6-7cm
   - ต้องลองระยะต่างๆ (20cm, 50cm, 80cm)

3. **ภาพมืดเกินไป** 💡
   - Mean 47-53 จาก 255
   - Corner detection อาจจะไม่แม่นยำ
   - → Calibration parameters ผิดพลาด

4. **OpenCV Calibration Algorithm Limitation**
   - Algorithm อาจจะไม่เหมาะกับ setup นี้
   - ต้องใช้ calibration tool อื่น (Kalibr, MATLAB)

5. **Distortion มากเกินไป**
   - IMX219 มี distortion สูง
   - Calibration อาจจะไม่ compensate ได้หมด

#### 🔄 งานถัดไป (แนะนำ):

**ตัวเลือก 1: ตรวจสอบ Hardware Setup** 🔧
1. ตรวจสอบการ mount กล้อง - ต้องขนานกัน 100%
2. วัดระยะห่างจริงระหว่างกล้อง
3. ตรวจสอบว่ากล้องทั้งสองชี้ไปทิศทางเดียวกัน

**ตัวเลือก 2: ปรับปรุงคุณภาพภาพ** 💡
1. เพิ่มแสงสว่าง (LED หลายดวง)
2. ลองใช้ gamma correction เพื่อเพิ่มความสว่าง
3. ใช้ระยะ chessboard ต่างๆ (20-80 cm)

**ตัวเลือก 3: ใช้ Calibration Tool อื่น** 🛠️
1. **Kalibr**: ROS-based, มี GUI ดี
2. **MATLAB Stereo Camera Calibrator**: Industry standard
3. **Camera Calibration Toolbox for MATLAB**: ละเอียดมาก

**ตัวเลือก 4: ใช้ Preset Calibration** ⚡
1. หา calibration parameters จากผู้ใช้ IMX219 Stereo คนอื่น
2. ใช้ manufacturer calibration (ถ้ามี)
3. ทำ manual calibration

#### 📝 สรุปบทเรียน:

**สิ่งที่เรียนรู้**:
- ✅ Individual camera calibration ทำได้ดี (RMS < 0.35)
- ✅ Corner detection ทำงานได้ดี (82/84 pairs)
- ✅ AUTO-CAPTURE mode ทำงานสมบูรณ์
- ✅ **Hardware mounting ไม่มีปัญหา** - ยืนยันแล้วว่ากล้องบน PCB เดียวกัน, baseline=60mm
- ❌ Stereo calibration ล้มเหลวอย่างสิ้นเชิง
- ❌ ภาพมืดเกินไป (mean 47-53) - **สาเหตุหลักที่เป็นไปได้**
- ❌ OpenCV algorithm อาจไม่เหมาะกับ IMX219 stereo setup

**ปัญหาที่ยังไม่แก้ได้**:
- ❌ Baseline 1.2-1.9m (error >2000%)
- ❌ Focal length ไม่สอดคล้อง (กล้องเดียวกัน แต่ต่างกัน 2 เท่า)
- ❌ Rotation 37° (ควรใกล้ 0°)
- ❌ Rectification ไม่ทำงาน (แสดงเฉพาะเส้น)

**ข้อมูลสำคัญ**:
- Calibration images: 80 คู่ (82 valid pairs)
- Resolution: 1280x720
- Gain: 10
- Pattern: 6x6 inner corners, 25mm
- ระยะ board: 41-43 cm
- ภาพเยื้อง: ~1 นิ้ว (2.5 cm) - ปกติ
- ภาพมืด: mean 47-53 (จาก 255)
- **Baseline จริง (วัดจากฮาร์ดแวร์): 60mm** ✅
- **Baseline ที่ได้จาก calibration: 1219mm** ❌ (error 2000%!)

**คำแนะนำ** (อัพเดทหลังได้ข้อมูล hardware):
1. ✅ ~~ตรวจสอบ hardware mounting~~ - **ยืนยันแล้ว: baseline = 60mm, บน PCB เดียวกัน**
2. 💡 **เพิ่มแสงสว่าง** - ภาพมืดมาก (mean 47-53) อาจส่งผลต่อ calibration
3. 🛠️ **ลองใช้ calibration tool อื่น** - Kalibr, MATLAB, หรือ Camera Calibration Toolbox
4. 📐 **ลองระยะ chessboard ต่างๆ** - ใกล้ขึ้น (20-30cm) หรือไกลขึ้น (60-80cm)
5. 🔬 **ตรวจสอบ distortion model** - IMX219 อาจต้องใช้ rational model
6. 📊 **ลองใช้ภาพที่สว่างกว่า** - brighten images หรือ capture ใหม่ด้วย gain สูงขึ้น

---

### วันที่ 2025-10-15 (Session 6: Calibration Round 2 - Resolution Fix 🔧)

#### ✅ งานที่เสร็จแล้ว:

1. **แก้ไข SQUARE_SIZE จาก 20mm → 25mm** ✅
   - Pattern จริง: 7x7 ช่อง (6x6 inner corners, **25mm x 25mm**)
   - อัพเดททั้ง `capture_calibration_images.py` และ `calibrate_stereo.py`
   - Code settings: `CHESSBOARD_SIZE = (6, 6)`, `SQUARE_SIZE = 25`

2. **ปรับ Camera Settings สำหรับ Lighting** ✅
   - เพิ่ม exposure และ gain control:
     - `aelock=true awblock=true` - ล็อค auto-exposure และ white balance
     - `exposuretimerange="33333333 33333333"` - ล็อค exposure time (33ms)
     - `gainrange="10 10"` - ล็อค gain ที่ 10 (สว่างเพียงพอ)
   - แก้ปัญหา: กล้องขวา detection ต่ำ → เพิ่ม gain จาก 4 → 8 → 10

3. **ลด Resolution เพื่อ Depth of Field** ✅
   - เปลี่ยนจาก 1920x1080 → **1280x720 @ 60fps**
   - เป้าหมาย: ให้ focus ชัดลึกขึ้น → detection ดีขึ้น
   - ผลลัพธ์: Detection ดีขึ้น แต่พบปัญหา rectification

4. **Calibration Attempt #1-3** (ใช้ display resolution 640x360)
   - ❌ **ปัญหา**: Stereo RMS สูงมาก (25-70 pixels)
   - ❌ **Baseline ผิดปกติ**: 1.9m → 7.1m → 37cm
   - ⚠️ **Valid pairs น้อย**: 7-56 คู่จาก 60+ ที่จับได้
   - **สาเหตุ**: กล้องขวา detection ไม่ดี + calibration parameters ผิดพลาด

5. **แก้ไข Bug ในโค้ด Calibration** ✅
   - **ปัญหา**: `zip(valid_left, valid_right)` ใช้ index ไม่ตรงกัน
   - **วิธีแก้**: ใช้ dictionary matching แทน zip
   - แก้ไขใน `calibrate_stereo.py` บรรทัด 309-333

6. **Calibration Attempt #4** (1280x720, gain=10, 56 valid pairs)
   - ✅ Left RMS: **0.15 pixels** (ดีเยี่ยม!)
   - ✅ Right RMS: **0.13 pixels** (ดีเยี่ยม!)
   - ⚠️ Stereo RMS: **25.23 pixels** (ยังสูงเกินไป!)
   - ⚠️ Baseline: **371.86 mm** (ดีขึ้นมาก แต่ยังสูงไป - ควร 60-70mm)
   - ใช้เวลา calibration: ~2-3 นาที

7. **ทดสอบ verify_calibration.py - พบปัญหา Resolution Mismatch** ⚠️
   - **ปัญหา**: ภาพเป็นสีน้ำตาล/ดำ แสดงแต่เส้น epipolar lines
   - **สาเหตุ**:
     - Calibration ทำที่ **640x360** (display resolution หลัง downscale)
     - แต่ rectification maps ถูกสร้างจาก detection บนภาพ 640x360
     - ทำให้ rectification ไม่ตรงกับภาพจริง (1280x720)
   - **การทดสอบ**: สร้าง `test_camera_raw.py` → **กล้องทำงานปกติ!**
   - **สรุป**: ปัญหาอยู่ที่ rectification maps ไม่ใช่กล้อง

8. **แก้ไข Resolution Mismatch - Final Fix** ✅
   - **วิธีแก้**: บันทึกภาพที่ **1280x720** (full resolution) แทน 640x360
   - แก้ไข `capture_calibration_images.py`:
     - Resize gray images กลับเป็น 1280x720 ก่อนบันทึก
     - Detection ยังทำที่ 640x360 (เร็ว)
     - บันทึกที่ 1280x720 (calibration ถูกต้อง)
   - แก้ไข `verify_calibration.py`:
     - CAPTURE: 1280x720
     - DISPLAY: 640x360
     - เพิ่ม exposure/gain settings เหมือน capture
   - ลบภาพ calibration เก่าเรียบร้อย

#### 🔄 งานถัดไป:

1. **จับภาพ Calibration Images ใหม่** (Resolution ถูกต้อง)
   - ใช้ AUTO-CAPTURE mode
   - ภาพจะถูกบันทึกที่ 1280x720
   - ระยะ: 41-43 cm
   - เป้าหมาย: 20-30 คู่

2. **Calibrate ใหม่**
   - รัน `python3 calibrate_stereo.py`
   - คาดหวัง: Stereo RMS < 1.0 pixels, Baseline ~60-70mm

3. **Verify Calibration**
   - รัน `python3 verify_calibration.py`
   - ควรเห็นภาพชัดเจน + epipolar lines

4. **Depth Map Applications**
   - เมื่อ calibration สำเร็จ

#### 📝 สรุปปัญหาและบทเรียน:

**ปัญหาหลัก**:
- ❌ **Resolution mismatch**: Calibration ใช้ display resolution (640x360) แต่ควรใช้ capture resolution (1280x720)
- ❌ **Lighting**: กล้องขวา detection ต่ำ → แก้ด้วย gain=10
- ❌ **SQUARE_SIZE ผิด**: ใช้ 20mm แทน 25mm จริง
- ❌ **Bug ใน code**: zip() ใช้ index ผิด

**การแก้ไข**:
- ✅ บันทึกภาพที่ full resolution (1280x720)
- ✅ Detection ที่ display resolution (640x360) - เพื่อความเร็ว
- ✅ ล็อค exposure/gain = 10
- ✅ แก้ไข matching algorithm ใช้ dictionary

**Settings ที่ถูกต้อง**:
- Pattern: 7x7 ช่อง (6x6 inner corners, 25mm)
- Capture: 1280x720 @ 60fps
- Display: 640x360 (detection + แสดงผล)
- Exposure: 33333333 ns (33ms)
- Gain: 10
- ระยะ board: 41-43 cm

---

### วันที่ 2025-10-15 (Session 5: Auto-Capture & Performance Optimization! 🎉)

#### ✅ งานที่เสร็จแล้ว:

1. **ทดสอบ Pattern 20x29 ช่อง (ล้มเหลว)**
   - Pattern แรก: 20x29 ช่อง (10mm ต่อช่อง) สีชมพู-ดำ
   - ปัญหา: กล้องขวาตรวจจับไม่ได้ (out of focus, pattern ใหญ่เกินไป)
   - แก้ไข code ให้แสดงและบันทึกเป็น grayscale แล้ว
   - กล้องซ้ายตรวจจับได้ แต่กล้องขวาไม่ได้

2. **เปลี่ยนเป็น Pattern 7x7 ช่อง (สำเร็จ! ✅)**
   - Pattern ใหม่: **7x7 ช่อง = 6x6 inner corners**
   - ขนาดช่อง: **20mm x 20mm**
   - แก้ไข code ทั้ง 2 ไฟล์:
     - `capture_calibration_images.py`: `CHESSBOARD_SIZE = (6, 6)`, `SQUARE_SIZE = 20`
     - `calibrate_stereo.py`: `CHESSBOARD_SIZE = (6, 6)`, `SQUARE_SIZE = 20`
   - **ผลลัพธ์**: ✅ **ตรวจจับสำเร็จในครั้งแรก!**
   - ทั้งกล้องซ้ายและขวาตรวจจับ pattern ได้พร้อมกัน
   - สถานะแสดง "READY - Press 'c' or SPACE" (สีเขียว)

3. **แก้ไข Code รองรับ Grayscale Display & Saving**
   - ✅ `capture_calibration_images.py`:
     - แสดงภาพเป็น grayscale (แปลงจาก gray เป็น BGR เพื่อวาด overlays)
     - บันทึกภาพเป็น grayscale
     - แก้ปัญหาสีชมพูจาก LED light case → เป็นสีเทา/ขาว

4. **Optimization สำหรับ Real-time Performance** ✅
   - **ปัญหา**: ภาพ lag มาก รอ 30 วินาทีถึงจะเห็นการเปลี่ยนแปลง
   - **วิธีแก้**:
     - ลด display resolution: 960x540 → **640x360**
     - Detection ทุก 2 frames แทนที่จะทุก frame
     - Temporal filtering (majority voting) จาก 3 frames ล่าสุด
     - Detection threshold: 2/3 frames
   - **ผลลัพธ์**: ✅ ภาพเร็วขึ้นมาก, lag ลดลงเหลือ ~1-2 วินาที

5. **แก้ไข Detection กระพริบ (Flickering)** ✅
   - **ปัญหา**: Detection หลุดๆ ติดๆ แม้ภาพนิ่งแล้ว
   - **วิธีแก้**:
     - เพิ่ม detection history (3 frames)
     - Majority voting - ต้อง detect 2/3 ครั้งถึงจะถือว่าพบ
     - แสดง confidence level: `LEFT: OK (3/3)`
     - Visual feedback ชัดเจน: Green banner + countdown
   - **ผลลัพธ์**: ✅ Detection มั่นคง ไม่กระพริบ

6. **แก้ปัญหา Capture หลัง Flash** ✅
   - **ปัญหา**: หลัง flash สีขาว กล้องซ้าย auto-exposure ปรับตัวไม่ทัน
   - **วิธีแก้**: Skip 5 frames หลัง flash เพื่อให้กล้องปรับตัว
   - **ผลลัพธ์**: ✅ Capture ได้ต่อเนื่องไม่มีปัญหา

7. **เพิ่ม AUTO-CAPTURE MODE** ✅ 🎯
   - **ฟีเจอร์**:
     - Auto-capture ทุก 3 วินาที (90 frames) เมื่อ detect pattern ได้
     - แสดง countdown timer บนหน้าจอ
     - หยุดอัตโนมัติทุก 10 ภาพ ให้เปลี่ยนมุม board
     - Resume ด้วย 'c' หรือ SPACE
     - Toggle pause ด้วย 'p'
   - **Controls**:
     - `c` / SPACE: Manual capture / Resume
     - `p`: Toggle pause
     - `d`: Delete last capture
     - `q` / ESC: Quit
   - **ผลลัพธ์**: ✅ ไม่ต้องกดปุ่ม - แค่วาง board ระบบจับเอง!

8. **Reboot และทดสอบกล้อง**
   - ✅ Reboot เครื่องเพื่อโหลด device tree
   - ✅ ทดสอบ GStreamer: กล้องทั้งสองทำงานปกติ
   - ✅ รัน `capture_calibration_images.py` สำเร็จ
   - ✅ Auto-capture ทำงานได้สมบูรณ์

#### 🔄 งานถัดไป:
1. **จับภาพ Calibration Images** (พร้อมแล้ว - ใช้ AUTO-CAPTURE)
   - ✅ Auto-capture mode พร้อมใช้งาน
   - จับภาพ 20-30 คู่จากมุมและระยะต่างๆ
   - ครอบคลุมทุกพื้นที่ของภาพ (center, corners, edges)
   - ใช้มุมต่างๆ (ตรง, เอียง, หมุน)
   - **Workflow**: วาง board → รอ 3 วินาที → auto-capture → เปลี่ยนมุม → repeat

2. **Calibrate กล้อง Stereo**
   - รัน `python3 calibrate_stereo.py`
   - ตรวจสอบ RMS error < 1.0 pixels

3. **ตรวจสอบคุณภาพ Calibration**
   - รัน `python3 verify_calibration.py`
   - ตรวจสอบ epipolar lines ว่าตรงกัน

4. **พัฒนา Depth Map Applications**
   - Real-time depth estimation
   - Distance measurement
   - 3D reconstruction

#### 📝 หมายเหตุสำคัญ:

**✅ Pattern ที่ใช้สำเร็จ**:
- ขนาด: **7x7 ช่อง** (6x6 inner corners)
- ขนาดช่อง: **20mm x 20mm**
- Code settings: `CHESSBOARD_SIZE = (6, 6)`, `SQUARE_SIZE = 20`
- แสดงและบันทึกเป็น **grayscale images**
- ตรวจจับได้ทั้งสองกล้องพร้อมกัน ✅

**❌ Pattern ที่ไม่สำเร็จ**:
- Pattern แรก: 20x29 ช่อง (10mm ต่อช่อง)
- ปัญหา: ช่องเล็กเกินไป, pattern ใหญ่เกินไป, กล้องขวา out of focus

**บทเรียน**:
- Pattern ขนาดกลาง (7x7 ช่อง, 20mm, 6x6 inner corners) เหมาะสมที่สุดสำหรับกล้อง stereo
- Grayscale conversion แก้ปัญหาสีผิดปกติจาก LED light
- ต้องให้ทั้งสองกล้องเห็น pattern ครบทั้งหมด (ไม่ตัดขอบ)

---

### วันที่ 2025-10-14 (Session 4: Code Quality & Calibration Testing)

#### ✅ งานที่เสร็จแล้ว:

1. **แก้ไข Lint Errors**
   - แก้ไข PEP 8 style issues ในไฟล์ทั้งหมด
   - `calibrate_stereo.py` - 16 errors fixed
   - `capture_calibration_images.py` - 3 errors fixed
   - `stereo_view.py` - 3 errors fixed
   - `test_stereo_camera.py` - 2 errors fixed
   - `verify_calibration.py` - 5 errors fixed
   - ✅ ทุกไฟล์ผ่าน flake8 linter (0 errors)

2. **ทดสอบกล้องหลังแก้ไข Lint**
   - สร้าง `test_single_camera.py` - ทดสอบกล้องทีละตัว
   - ✅ กล้องขวา (sensor-id=1): ทำงานปกติ
   - ✅ กล้องซ้าย (sensor-id=0): ทำงานปกติ
   - ✅ กล้องทั้งสองพร้อมกัน: ทำงานปกติ (stereo_view.py, test_stereo_camera.py)
   - **ผลการทดสอบ**: การแก้ไข lint ไม่กระทบการทำงานของโปรแกรม ✅

3. **เริ่มทดสอบ Calibration**
   - รัน `capture_calibration_images.py` สำเร็จ - กล้องเปิดได้
   - ⚠️ **ปัญหา**: Chessboard detection ไม่ทำงาน
   - **สาเหตุ**: Chessboard pattern มีสีผิด (ชมพู-น้ำเงิน แทนที่จะเป็น ขาว-ดำ)
   - **Contrast ต่ำเกินไป**: OpenCV ต้องการ high contrast (ขาว-ดำ หรือ ดำ-ขาว)

---

### วันที่ 2025-10-14 (Session 3: Ready to Apply Fix)

#### ✅ พร้อมแก้ไขปัญหา Boot Configuration:

**ปัญหาที่ระบุไว้**: FDT line อยู่ใน label `primary-backup` แต่ระบบ boot จาก label `primary` (DEFAULT)

**วิธีแก้**:
สคริปต์ `fix_boot_config.sh` พร้อมใช้งานแล้ว - จะเพิ่ม FDT line ใน `primary` label

**ขั้นตอนที่ต้องทำ** (ต้องรันใน terminal เพราะต้องใช้ sudo):
```bash
cd /home/jay/projects/stereo_camera
sudo ./fix_boot_config.sh
sudo reboot
```

หรือรัน commands ทีละขั้นตอน:
```bash
# 1. Backup config
sudo cp /boot/extlinux/extlinux.conf /boot/extlinux/extlinux.conf.backup-$(date +%Y%m%d-%H%M%S)

# 2. เพิ่ม FDT line ใน primary label
sudo sed -i '/LABEL primary/,/APPEND/ {
    /LINUX \/boot\/Image/a\      FDT /boot/dtb/kernel_tegra234-p3768-0000+p3767-0005-nv-super-imx219.dtb
}' /boot/extlinux/extlinux.conf

# 3. ตรวจสอบว่าแก้ไขสำเร็จ
sudo grep -A 6 "LABEL primary" /boot/extlinux/extlinux.conf | head -8

# 4. Reboot
sudo reboot
```

#### 🔄 งานถัดไป:
1. **✅ สร้างสคริปต์ fix_boot_config.sh** - เสร็จแล้ว
2. **⏳ รันสคริปต์และ Reboot** - รอ user รันใน terminal (ต้องใช้ sudo)
3. **รัน Calibration** - จับภาพและ calibrate กล้อง stereo
4. **สร้าง Depth Map Script** - สำหรับการประมวลผล depth estimation

---

### วันที่ 2025-10-14 (Session 2: Debug Boot Configuration)

#### ❌ ปัญหาที่พบหลัง Reboot:

**ปัญหา**: หลัง reboot ยังได้ error "No cameras available"

**การตรวจสอบ**:
1. ✅ ไฟล์ merged DTB มีอยู่: `/boot/dtb/kernel_tegra234-p3768-0000+p3767-0005-nv-super-imx219.dtb` (257KB)
2. ✅ ไฟล์ DTB มี IMX219 nodes อยู่ (ตรวจสอบด้วย `dtc`)
3. ❌ Device tree ไม่มี IMX219 ใน `/proc/device-tree`
4. ❌ ไม่มี `/dev/video*` devices
5. ❌ I2C bus 10 ไม่พบ

**สาเหตุที่แท้จริง**:
FDT line อยู่ใน label `primary-backup` แต่ระบบ boot จาก label `primary` (DEFAULT) ซึ่ง**ไม่มี FDT line**

**extlinux.conf ปัจจุบัน**:
```
DEFAULT primary         ← Boot จาก label นี้

LABEL primary           ← ไม่มี FDT line!
      MENU LABEL primary kernel
      LINUX /boot/Image
      INITRD /boot/initrd
      APPEND ${cbootargs} ...

LABEL primary-backup    ← FDT line อยู่ที่นี่ แต่ไม่ได้ใช้!
      MENU LABEL primary kernel backup
      LINUX /boot/Image
      FDT /boot/dtb/kernel_tegra234-p3768-0000+p3767-0005-nv-super-imx219.dtb
      INITRD /boot/initrd
      APPEND ${cbootargs} ...
```

#### ✅ วิธีแก้:

**สร้างสคริปต์**: `fix_boot_config.sh`
- เพิ่ม FDT line ใน `primary` label (ที่ถูก boot จริง)
- Backup config ก่อนแก้ไข
- แสดงผลการแก้ไข

---

### วันที่ 2025-10-14 (Session 1: Calibration Scripts)

#### ✅ งานที่เสร็จแล้ว:

1. **สร้าง Calibration Scripts**
   - `capture_calibration_images.py` - จับภาพ stereo สำหรับ calibration พร้อม chessboard detection
   - `calibrate_stereo.py` - คำนวณ camera parameters และ rectification maps
   - `verify_calibration.py` - ตรวจสอบคุณภาพ calibration ด้วย epipolar lines
   - `CALIBRATION_GUIDE.md` - คู่มือ calibration แบบละเอียด

2. **อัพเดทเอกสาร**
   - `README.md` - เพิ่มส่วน Stereo Camera Calibration
   - อัพเดต Project Files section

3. **เตรียมความพร้อมสำหรับ Calibration**
   - Chessboard pattern: 7x7 inner corners (8x8 squares)
   - Square size: 25mm x 25mm
   - LED lighting พร้อมใช้งาน

---

### วันที่ 2025-10-08

#### ✅ งานที่เสร็จแล้ว (Setup เริ่มต้น):

1. **ติดตั้งและตรวจสอบระบบ**
   - ตรวจสอบ JetPack version: R36.4.4
   - ตรวจสอบ kernel version: 5.15.148-tegra
   - ยืนยันว่ามี nvidia-l4t-camera package ติดตั้งแล้ว

2. **ค้นหา Device Tree Overlay**
   - พบไฟล์ที่จำเป็น: `/boot/tegra234-p3767-camera-p3768-imx219-dual.dtbo`
   - พบ DTB file: `/boot/kernel_tegra234-p3768-0000+p3767-0005-nv-super.dtb`

3. **กำหนดค่า Boot Configuration**
   - แก้ไขไฟล์: `/boot/extlinux/extlinux.conf`
   - เพิ่มบรรทัด FDT overlay:
     ```
     FDT /boot/dtb/kernel_tegra234-p3768-0000+p3767-0005-nv-super.dtb /boot/tegra234-p3767-camera-p3768-imx219-dual.dtbo
     ```
   - สร้าง backup: `/boot/extlinux/extlinux.conf.backup`, `/boot/extlinux/extlinux.conf.backup2`
   - **แก้ไขปัญหา**: DTB path ต้องเป็น `/boot/dtb/kernel_...` ไม่ใช่ `/boot/kernel_...`

4. **สร้างสคริปต์ทดสอบและติดตั้ง**
   - `test_stereo_camera.py` - สคริปต์ทดสอบกล้อง stereo
   - `enable_imx219_stereo.sh` - สคริปต์ติดตั้ง device tree overlay

5. **แก้ไขปัญหา Device Tree Overlay ไม่โหลด** (Reboot ครั้งที่ 2)
   - **ปัญหา**: หลัง reboot ครั้งที่ 2 ยังได้ error "No cameras available"
   - **สาเหตุ**: การใส่ overlay แบบ 2 path ใน FDT line ไม่ทำงาน
   - **วิธีแก้**: ใช้ `fdtoverlay` command เพื่อ merge overlay กับ base DTB

   ```bash
   # 1. Merge overlay กับ base DTB
   fdtoverlay -i /boot/dtb/kernel_tegra234-p3768-0000+p3767-0005-nv-super.dtb \
              -o /tmp/merged.dtb \
              /boot/tegra234-p3767-camera-p3768-imx219-dual.dtbo

   # 2. Copy merged DTB ไปที่ /boot
   sudo cp /tmp/merged.dtb /boot/dtb/kernel_tegra234-p3768-0000+p3767-0005-nv-super-imx219.dtb

   # 3. แก้ไข extlinux.conf ให้ชี้ไปที่ merged DTB
   FDT /boot/dtb/kernel_tegra234-p3768-0000+p3767-0005-nv-super-imx219.dtb
   ```

   - สร้าง backup: `/boot/extlinux/extlinux.conf.backup3`
   - ไฟล์ merged DTB: `/boot/dtb/kernel_tegra234-p3768-0000+p3767-0005-nv-super-imx219.dtb` (257KB)
   - ตรวจสอบว่ามี IMX219 ใน merged DTB แล้ว ✅

6. **Reboot ครั้งที่ 3 และทดสอบกล้อง** ✅
   - Reboot เครื่องเพื่อโหลด merged DTB
   - พบ `/dev/video0` และ `/dev/video1` ✅
   - ทดสอบกล้องครั้งแรก:
     - **sensor-id=0 (กล้องซ้าย)**: ✅ ทำงานปกติ - แสดงภาพ 30 frames สำเร็จ
     - **sensor-id=1 (กล้องขวา)**: ❌ Error "Failed to create CaptureSession"

7. **ตรวจสอบและแก้ไขปัญหากล้องขวา** ✅
   - ตรวจสอบ I2C bus:
     - Bus 9: พบ `0x10` (imx219 9-0010) ← กล้องซ้าย
     - Bus 10: พบ `0x10` (imx219 10-0010) ← กล้องขวา
   - ตรวจสอบ kernel log:
     ```
     imx219 9-0010: tegracam sensor driver:imx219_v2.0.6  ← กล้องตัวที่ 1
     imx219 10-0010: tegracam sensor driver:imx219_v2.0.6 ← กล้องตัวที่ 2
     ```
   - **ทดสอบกล้องทีละตัวอีกครั้ง**:
     - **sensor-id=0 (กล้องซ้าย)**: ✅ ทำงานปกติ - "Done Success"
     - **sensor-id=1 (กล้องขวา)**: ✅ ทำงานปกติ - "Done Success"

   **ผลการทดสอบ**:
   ```bash
   # กล้องซ้าย - สำเร็จ
   gst-launch-1.0 nvarguscamerasrc sensor-id=0 num-buffers=30 ! 'video/x-raw(memory:NVMM),width=1920,height=1080,framerate=30/1' ! nvvidconv ! xvimagesink
   # Output: Done Success ✅

   # กล้องขวา - สำเร็จ
   gst-launch-1.0 nvarguscamerasrc sensor-id=1 num-buffers=30 ! 'video/x-raw(memory:NVMM),width=1920,height=1080,framerate=30/1' ! nvvidconv ! xvimagesink
   # Output: Done Success ✅
   ```

   **สรุป**: กล้อง stereo ทั้งสองตัวทำงานได้แล้ว! ปัญหาเดิมอาจเกิดจากการทดสอบครั้งแรกที่กล้องยังไม่พร้อม

#### 🔄 งานถัดไป (จากวันที่ 2025-10-08):
- ✅ สร้าง calibration scripts (เสร็จแล้ว วันที่ 2025-10-14)

## ไฟล์ที่สร้างขึ้น

### Boot Configuration Script (เพิ่มเมื่อ 2025-10-14)

#### fix_boot_config.sh
**ตำแหน่ง**: `/home/jay/projects/stereo_camera/fix_boot_config.sh`

**คำอธิบาย**: สคริปต์สำหรับแก้ไข boot configuration ให้ใช้ merged DTB ที่ถูกต้อง
- Backup extlinux.conf ก่อนแก้ไข (timestamped)
- เพิ่ม FDT line ใน `primary` label (ที่ระบบ boot จริง)
- แสดงผล configuration ที่แก้ไขแล้ว

**วิธีใช้**:
```bash
cd /home/jay/projects/stereo_camera
./fix_boot_config.sh
sudo reboot
```

**หมายเหตุ**: แก้ปัญหา FDT line อยู่ใน `primary-backup` label ที่ไม่ได้ถูก boot

---

### Calibration Scripts (เพิ่มเมื่อ 2025-10-14)

#### 1. capture_calibration_images.py
**ตำแหน่ง**: `/home/jay/projects/stereo_camera/capture_calibration_images.py`

**คำอธิบาย**: สคริปต์สำหรับจับภาพ stereo เพื่อใช้ในการ calibration
- เปิดกล้อง stereo ทั้งสองตัวพร้อมกัน
- ตรวจจับ chessboard pattern แบบ real-time (7x7 inner corners)
- แสดงสถานะ detection บนหน้าจอ (เขียว = พบ pattern, แดง = ไม่พบ)
- บันทึกภาพเป็นคู่ (left + right) พร้อมกัน
- วาด chessboard corners บนภาพ
- กด 'c' หรือ SPACE เพื่อ capture, 'd' เพื่อลบภาพล่าสุด
- บันทึกไปที่ `calibration_images/left/` และ `calibration_images/right/`

**วิธีใช้**:
```bash
python3 capture_calibration_images.py
```

**แนะนำ**: จับภาพ 20-30 คู่จากมุมและระยะต่างๆ

#### 2. calibrate_stereo.py
**ตำแหน่ง**: `/home/jay/projects/stereo_camera/calibrate_stereo.py`

**คำอธิบาย**: สคริปต์สำหรับ calibrate กล้อง stereo
- โหลดภาพจาก `calibration_images/`
- ตรวจจับ chessboard corners ในแต่ละภาพ
- คำนวณ intrinsic parameters (focal length, optical center, distortion)
- คำนวณ extrinsic parameters (rotation, translation, baseline)
- สร้าง rectification maps
- บันทึกผลลัพธ์เป็น `stereo_calibration.npz` และ `calibration_report.txt`

**วิธีใช้**:
```bash
python3 calibrate_stereo.py
```

**Output**:
- `stereo_calibration.npz` - Binary calibration data
- `calibration_report.txt` - Human-readable report

#### 3. verify_calibration.py
**ตำแหน่ง**: `/home/jay/projects/stereo_camera/verify_calibration.py`

**คำอธิบาย**: สคริปต์สำหรับตรวจสอบคุณภาพการ calibration
- โหลด calibration data จาก `stereo_calibration.npz`
- แสดง rectified video แบบ real-time
- วาดเส้น epipolar lines (แนวนอน) สำหรับตรวจสอบความตรง
- กด 'l' เพื่อ toggle lines, 's' เพื่อบันทึกภาพ
- ตรวจสอบว่าวัตถุอยู่ในแนวระดับเดียวกันในทั้งสองภาพ

**วิธีใช้**:
```bash
python3 verify_calibration.py
```

**สิ่งที่ต้องตรวจสอบ**:
- เส้นแนวนอนควรผ่านจุดเดียวกันในทั้งสองภาพ
- วัตถุควรอยู่ในระดับความสูงเดียวกัน
- ไม่มี vertical misalignment

#### 4. CALIBRATION_GUIDE.md
**ตำแหน่ง**: `/home/jay/projects/stereo_camera/CALIBRATION_GUIDE.md`

**คำอธิบาย**: คู่มือ calibration แบบละเอียดครบถ้วน
- วิธีเตรียมอุปกรณ์
- ขั้นตอนการจับภาพ (best practices)
- การทำความเข้าใจ calibration parameters
- Troubleshooting และ tips
- คำอธิบาย intrinsic/extrinsic parameters

### Testing Scripts

#### test_single_camera.py (เพิ่มเมื่อ 2025-10-14)
**ตำแหน่ง**: `/home/jay/projects/stereo_camera/test_single_camera.py`

**คำอธิบาย**: สคริปต์สำหรับทดสอบกล้องทีละตัว (แก้ปัญหากล้องค้างเมื่อเปิดพร้อมกัน)
- ทดสอบกล้องขวา (sensor-id=1) ก่อน
- จากนั้นทดสอบกล้องซ้าย (sensor-id=0)
- แสดงจำนวน frame ที่จับได้
- รอ 2 วินาทีระหว่างการทดสอบ
- แสดงสรุปผลการทดสอบ

**วิธีใช้**:
```bash
python3 test_single_camera.py
```

**Controls**:
- กด 'q' เพื่อปิดกล้องปัจจุบันและไปกล้องถัดไป

**ข้อดี**:
- ไม่เปิด 2 กล้องพร้อมกัน → แก้ปัญหาค้าง
- ทดสอบทีละตัว → หากมีปัญหาจะรู้ว่ากล้องไหนมีปัญหา

#### test_stereo_camera.py (สร้างเมื่อ 2025-10-08)
**ตำแหน่ง**: `/home/jay/test_stereo_camera.py`

**คำอธิบาย**: สคริปต์ Python สำหรับทดสอบกล้อง stereo IMX219
- เปิดกล้องทั้งสองตัว (sensor-id=0 และ sensor-id=1)
- แสดงภาพจากกล้องซ้ายและขวาแบบ real-time
- กด 'q' เพื่อออก
- กด 's' เพื่อบันทึกรูปภาพ

**วิธีใช้**:
```bash
python3 test_stereo_camera.py
```

**Parameters ที่สามารถปรับได้**:
- `capture_width`: ความกว้างภาพที่ capture (default: 1920)
- `capture_height`: ความสูงภาพที่ capture (default: 1080)
- `display_width`: ความกว้างภาพที่แสดง (default: 960)
- `display_height`: ความสูงภาพที่แสดง (default: 540)
- `framerate`: FPS (default: 30)
- `flip_method`: หมุนภาพ (0-7)

### 2. enable_imx219_stereo.sh
**ตำแหน่ง**: `/home/jay/enable_imx219_stereo.sh`

**คำอธิบาย**: สคริปต์สำหรับติดตั้ง device tree overlay สำหรับกล้อง stereo
- Backup config เดิม
- ตรวจหา DTB file ที่ถูกต้อง
- เพิ่ม FDT overlay ลงใน boot config

**วิธีใช้**:
```bash
chmod +x enable_imx219_stereo.sh
sudo ./enable_imx219_stereo.sh
```

## ปัญหาที่พบและวิธีแก้

### ปัญหา 1: "No cameras available"
**สาเหตุ**: ไม่มี device tree overlay สำหรับกล้อง stereo ใน boot configuration

**วิธีแก้**: เพิ่ม FDT overlay ใน `/boot/extlinux/extlinux.conf`
```
FDT /boot/dtb/kernel_tegra234-p3768-0000+p3767-0005-nv-super.dtb /boot/tegra234-p3767-camera-p3768-imx219-dual.dtbo
```

### ปัญหา 3: DTB ไม่โหลด / กล้องไม่ detect หลัง reboot
**สาเหตุ**: DTB file path ผิด - ไฟล์จริงอยู่ใน `/boot/dtb/` ไม่ใช่ `/boot/`

**วิธีตรวจสอบ**:
```bash
# ตรวจสอบว่ามีกล้องบน I2C หรือไม่
i2cdetect -y -r 7

# ตรวจสอบว่ามี IMX219 ใน device tree หรือไม่
find /proc/device-tree -name "*imx219*"
```

**วิธีแก้**: แก้ path ใน `/boot/extlinux/extlinux.conf` จาก `/boot/kernel_...` เป็น `/boot/dtb/kernel_...`

### ปัญหา 4: Overlay ไม่ถูกนำไปใช้แม้ path ถูกต้อง
**สาเหตุ**: extlinux.conf ไม่รองรับ overlay แบบ 2 path ใน FDT line (บางเวอร์ชัน JetPack)

**อาการ**:
- ไม่มี camera nodes ใน device tree (`find /proc/device-tree -name "*imx219*"` ไม่พบอะไร)
- ยังได้ error "No cameras available" แม้ว่า DTB path ถูกต้อง

**วิธีแก้ที่ถูกต้อง**: ใช้ `fdtoverlay` เพื่อ merge overlay กับ base DTB ก่อน
```bash
# Merge overlay
fdtoverlay -i /boot/dtb/kernel_tegra234-p3768-0000+p3767-0005-nv-super.dtb \
           -o /tmp/merged.dtb \
           /boot/tegra234-p3767-camera-p3768-imx219-dual.dtbo

# Copy ไปที่ /boot
sudo cp /tmp/merged.dtb /boot/dtb/kernel_tegra234-p3768-0000+p3767-0005-nv-super-imx219.dtb

# ใช้ merged DTB ใน extlinux.conf
FDT /boot/dtb/kernel_tegra234-p3768-0000+p3767-0005-nv-super-imx219.dtb
```

### ปัญหา 5: FDT line อยู่ใน label ที่ไม่ถูก boot
**สาเหตุ**: FDT line อยู่ใน `primary-backup` label แต่ระบบ boot จาก `primary` label (DEFAULT)

**อาการ**:
- ไฟล์ merged DTB มีอยู่และมี IMX219 nodes
- แต่หลัง reboot ยังได้ error "No cameras available"
- `/proc/device-tree` ไม่มี IMX219 nodes
- extlinux.conf มี FDT line แต่อยู่ใน label ที่ไม่ได้ใช้

**วิธีตรวจสอบ**:
```bash
# ดู boot configuration
cat /boot/extlinux/extlinux.conf

# ตรวจสอบว่า DEFAULT boot จาก label ไหน
grep "DEFAULT" /boot/extlinux/extlinux.conf

# ตรวจสอบว่า label นั้นมี FDT line หรือไม่
grep -A 5 "LABEL primary" /boot/extlinux/extlinux.conf
```

**วิธีแก้**:
เพิ่ม FDT line ใน label ที่ถูก boot จริง (primary) ไม่ใช่ primary-backup

**ใช้สคริปต์**: `fix_boot_config.sh`
```bash
cd /home/jay/projects/stereo_camera
./fix_boot_config.sh
sudo reboot
```

### ปัญหา 2: ไม่มี /dev/video*
**สาเหตุ**: กล้อง CSI บน Jetson ใช้ NVIDIA Tegra camera subsystem ไม่ใช่ V4L2

**วิธีแก้**: ใช้ GStreamer pipeline แทน:
```python
gstreamer_pipeline(sensor_id=0)  # สำหรับกล้องซ้าย
gstreamer_pipeline(sensor_id=1)  # สำหรับกล้องขวา
```

## ขั้นตอนการ Calibration (2025-10-14)

### เตรียมความพร้อม
- ✅ Chessboard pattern: 7x7 inner corners (8x8 squares)
- ✅ Square size: 25mm x 25mm
- ✅ LED lighting สำหรับแสงสว่างที่สม่ำเสมอ
- ✅ Calibration scripts พร้อมใช้งาน

### ขั้นตอน Calibration

#### ขั้นที่ 1: Reboot เครื่อง
เพื่อให้ device tree โหลดกล้อง stereo:
```bash
sudo reboot
```

หลัง reboot ตรวจสอบกล้อง:
```bash
# ทดสอบกล้องซ้าย
gst-launch-1.0 nvarguscamerasrc sensor-id=0 num-buffers=30 ! \
  'video/x-raw(memory:NVMM),width=1920,height=1080,framerate=30/1' ! \
  nvvidconv ! xvimagesink

# ทดสอบกล้องขวา
gst-launch-1.0 nvarguscamerasrc sensor-id=1 num-buffers=30 ! \
  'video/x-raw(memory:NVMM),width=1920,height=1080,framerate=30/1' ! \
  nvvidconv ! xvimagesink
```

#### ขั้นที่ 2: จับภาพ Calibration Images
```bash
cd /home/jay/projects/stereo_camera
python3 capture_calibration_images.py
```

**Tips**:
- วาง chessboard ในตำแหน่งต่างๆ (center, corners, edges)
- ใช้มุมต่างๆ (ตรง, เอียง, หมุน)
- จับภาพ 20-30 คู่
- รอให้ทั้งสองกล้อง detect pattern (เห็นสถานะเป็นสีเขียว)
- กด 'c' หรือ SPACE เพื่อจับภาพ

#### ขั้นที่ 3: Calibrate
```bash
python3 calibrate_stereo.py
```

ผลลัพธ์ที่ได้:
- `stereo_calibration.npz` - Calibration data
- `calibration_report.txt` - Detailed report

#### ขั้นที่ 4: ตรวจสอบคุณภาพ
```bash
python3 verify_calibration.py
```

ตรวจสอบว่า:
- เส้นแนวนอนตรงกัน
- วัตถุอยู่ในระดับความสูงเดียวกัน
- RMS error < 1.0 pixels

### ขั้นตอนต่อไป (หลัง Calibration สำเร็จ)

#### 1. Depth Map Generation
- สร้าง real-time depth maps
- ใช้ stereo matching algorithms (BM, SGBM)
- แสดงผล depth map เป็น color-coded image

#### 2. 3D Reconstruction
- สร้าง point clouds จาก depth maps
- 3D visualization
- Surface reconstruction

#### 3. Distance Measurement
- วัดระยะทางของวัตถุ
- Object detection + distance
- Obstacle detection สำหรับหุ่นยนต์

#### 4. Stereo Vision Applications
- SLAM (Simultaneous Localization and Mapping)
- Visual odometry
- Autonomous navigation
- Augmented Reality

#### 5. Optimization
- ปรับ resolution และ framerate ให้เหมาะสม
- ใช้ CUDA acceleration สำหรับ stereo matching
- TensorRT สำหรับ neural networks
- Hardware-accelerated depth estimation

## Resources และเอกสารอ้างอิง

### Official Documentation
- [NVIDIA Jetson Linux Developer Guide](https://docs.nvidia.com/jetson/l4t/)
- [GStreamer on Jetson](https://developer.nvidia.com/embedded/learn/tutorials/first-picture-csi-usb-camera)

### Useful Commands
```bash
# ตรวจสอบกล้อง
v4l2-ctl -d /dev/media0 --list-devices
ls /dev/video*

# ทดสอบด้วย GStreamer
gst-launch-1.0 nvarguscamerasrc sensor-id=0 ! nvvidconv ! xvimagesink

# ดู kernel messages
dmesg | grep -i camera

# ตรวจสอบ I2C bus
i2cdetect -y -r 7
i2cdetect -y -r 9

# ดู device tree
cat /proc/device-tree/model
```

### GStreamer Pipeline for Jetson CSI Camera
```bash
nvarguscamerasrc sensor-id=0 !
  video/x-raw(memory:NVMM),width=1920,height=1080,framerate=30/1 !
  nvvidconv flip-method=0 !
  video/x-raw,width=960,height=540,format=BGRx !
  videoconvert !
  video/x-raw,format=BGR !
  appsink
```

## หมายเหตุ

- กล้อง IMX219 รองรับ resolution สูงสุด 3280x2464 @ 21fps
- สำหรับ stereo vision แนะนำใช้ 1920x1080 @ 30fps เพื่อ balance ระหว่าง quality และ performance
- ต้อง calibrate กล้อง stereo ก่อนใช้งาน depth estimation
- Jetson Orin Nano มี CUDA cores และ Tensor cores ที่ช่วย accelerate computer vision tasks

## Backup และ Restore

### Backup Config
```bash
sudo cp /boot/extlinux/extlinux.conf /boot/extlinux/extlinux.conf.backup
```

### Restore Config
```bash
sudo cp /boot/extlinux/extlinux.conf.backup /boot/extlinux/extlinux.conf
sudo reboot
```

---

**สร้างโดย**: Claude Code
**วันที่เริ่มโปรเจค**: 2025-10-08
**อัพเดทล่าสุด**: 2025-10-16

## สถานะล่าสุด

### 2025-10-16 (Late Evening): Phase 2 - IMU Investigation & Calibration Prep 🔄
**Status**: In Progress - เปลี่ยนแผนเป็น camera-only calibration

**สรุปวันนี้**:
1. ✅ **IMU Investigation Complete**
   - Scanned I2C/SPI buses ทั้งหมด - ไม่พบ ICM20948
   - สร้างเอกสาร `IMU_INVESTIGATION.md`
   - ตัดสินใจ: ดำเนินการต่อแบบ camera-only

2. ✅ **Calibration Tools Preparation**
   - เปลี่ยนจาก Kalibr → ROS2 camera_calibration
   - สร้าง `install_ros2_calibration.sh`
   - พร้อมติดตั้ง: camera_calibration, image_pipeline, stereo_image_proc

3. 📝 **Next Actions**:
   - รันสคริปต์ติดตั้ง calibration tools
   - เตรียม calibration target (chessboard 6x6 หรือ April tags)
   - เริ่ม stereo camera calibration

**Tools สร้างวันนี้**:
- `scan_imu.py` - I2C scanner for ICM20948
- `scan_all_i2c.py` - Full I2C bus scanner
- `IMU_INVESTIGATION.md` - IMU investigation report
- `install_ros2_calibration.sh` - ROS2 calibration installer

---

### 2025-10-16 (Evening): ROS2 + Visualization Complete! ✅
**Phase 1 เสร็จสมบูรณ์!**

**ความสำเร็จ**:
- ✅ ROS2 Humble ติดตั้งบน host (native installation)
- ✅ Stereo Camera Node ทำงานสมบูรณ์ @ 20 fps
- ✅ Topics ทั้งหมด publish ได้ (image_raw + camera_info)
- ✅ RViz2 และ rqt_image_view พร้อมใช้งาน
- ✅ Helper scripts สำหรับ start/stop system
- ✅ เอกสารครบถ้วน (ROS2_STEREO_GUIDE.md, VIEWER_GUIDE.md)

**ผลการทดสอบ**:
- Published: 7000+ stereo frame pairs
- Frame Rate: ~20 fps (stable)
- Resolution: 1280x720 BGR8
- Performance: No frame drops

---

### 2025-10-15 (Session 7)
❌ **Stereo Calibration Issues - Deep Investigation**

**ความคืบหน้า**:
- ✅ จับภาพ 80 คู่สำเร็จ (AUTO-CAPTURE mode ทำงานสมบูรณ์!)
- ✅ Individual camera calibration ดีเยี่ยม (RMS < 0.35 pixels)
- ❌ **Stereo calibration ล้มเหลวอย่างสิ้นเชิง**:
  - Stereo RMS: 19-27 pixels (ควร < 1.0)
  - Baseline: 1.2-1.9 เมตร (ควร 6-7 cm) - **Error >2000%!**
  - Focal length ไม่สอดคล้อง (กล้องเดียวกัน แต่ต่างกัน 2 เท่า)
  - Rotation: 37° (ควรใกล้ 0°)
- ❌ Rectification ใช้งานไม่ได้ (แสดงเฉพาะเส้น)

**สาเหตุที่เป็นไปได้**:
1. ✅ ~~กล้องไม่ parallel~~ → **ยืนยันแล้ว: baseline=60mm, บน PCB เดียวกัน**
2. 💡 **ภาพมืดเกินไป** (mean 47-53) → สาเหตุหลักที่เป็นไปได้สูง
3. 🛠️ **OpenCV algorithm** อาจไม่เหมาะกับ setup นี้
4. 📐 **ระยะ chessboard** อาจไม่เหมาะสม (41-43cm)

**แนะนำ** (อัพเดทแล้ว):
- ✅ ~~ตรวจสอบ hardware~~ - **ยืนยันแล้ว: 60mm baseline**
- 💡 **เพิ่มแสงสว่าง** - ปัญหาหลักที่เป็นไปได้
- 🛠️ ลองใช้ calibration tool อื่น (Kalibr, MATLAB)
- 📐 ลองระยะ chessboard ต่างๆ

### 2025-10-15 (Session 6)
✅ **Resolution Mismatch Fix Complete**
- แก้ไข SQUARE_SIZE 20→25mm
- แก้ไข resolution mismatch (บันทึก 1280x720, detect 640x360)
- Fix calibration matching bug

### 2025-10-15 (Session 5)
✅ **Auto-Capture Mode & Performance Optimization Complete! 🎉**

**ความสำเร็จ**:
- ✅ เปลี่ยนเป็น pattern **7x7 ช่อง** (6x6 inner corners, 20mm ต่อช่อง)
- ✅ แก้ไข code รองรับ grayscale display และ saving
- ✅ **ตรวจจับ pattern สำเร็จในครั้งแรก!**
- ✅ ทั้งกล้องซ้ายและขวาตรวจจับพร้อมกัน
- ✅ **Performance optimization**: ภาพเร็วขึ้น lag ลดเหลือ 1-2 วินาที
- ✅ **Temporal filtering**: Detection มั่นคง ไม่กระพริบ
- ✅ **Auto-capture mode**: จับอัตโนมัติทุก 3 วินาที
- ✅ **Smart pause**: หยุดทุก 10 ภาพเพื่อเปลี่ยนมุม board
- ✅ พร้อมจับภาพ calibration อย่างสมบูรณ์!

**Code changes**:
- `capture_calibration_images.py`:
  - `CHESSBOARD_SIZE = (6, 6)`, `SQUARE_SIZE = 20`
  - Display resolution: `640x360` (ลดจาก 960x540)
  - Detection every 2 frames (ไม่ใช่ทุก frame)
  - Temporal filtering (3-frame history)
  - Detection threshold: 2/3 frames
  - Auto-capture mode: ทุก 90 frames (~3 วินาที)
  - Pause every 10 captures
  - Confidence display: `LEFT: OK (3/3)`
  - แสดงและบันทึกเป็น grayscale
- `calibrate_stereo.py`:
  - `CHESSBOARD_SIZE = (6, 6)`, `SQUARE_SIZE = 20`

**Pattern ที่ทดสอบ**:
- ❌ Pattern 1: 20x29 ช่อง (10mm) - ล้มเหลว (ช่องเล็กเกิน, กล้องขวา out of focus)
- ✅ Pattern 2: 7x7 ช่อง (20mm, 6x6 inner corners) - **สำเร็จ!**

**Performance Optimization**:
- Display: 640x360 (ลดจาก 960x540)
- Detection: ทุก 2 frames + temporal filtering
- Lag: ลดจาก 30 วินาที → 1-2 วินาที
- Stability: Detection มั่นคง confidence 2/3 frames

**Auto-Capture Features**:
- ⏱️ Auto-capture ทุก 3 วินาที
- ⏸️ Pause ทุก 10 ภาพ
- 📊 Real-time confidence display
- 🎯 Countdown timer

🔄 **ต่อไป**: ใช้ auto-capture จับภาพ 20-30 คู่ → รัน calibration → verify

### 2025-10-14 (Session 4)
✅ **Code Quality & Calibration Testing**
- แก้ไข lint errors ทั้งหมด (29 errors fixed, 0 errors remaining)
- ทดสอบกล้องทั้งสองตัวหลังแก้ไข lint - ทำงานปกติ
- สร้าง test_single_camera.py สำหรับทดสอบกล้องทีละตัว
- ยืนยันว่าการแก้ไข lint ไม่กระทบการทำงานของโปรแกรม
- เริ่มทดสอบ calibration - พบปัญหา chessboard detection
- ⚠️ **ปัญหา**: Chessboard pattern สีผิด (ชมพู-น้ำเงิน) ต้องเป็น ขาว-ดำ

### 2025-10-14 (Session 3)
⏳ **พร้อมแก้ไขปัญหา Boot Configuration** (ยังไม่ได้รัน)
- ระบุสาเหตุ: FDT line อยู่ใน `primary-backup` แต่ระบบ boot จาก `primary` label
- ✅ สร้าง `fix_boot_config.sh` เพื่อแก้ไข
- ⏳ **ขั้นตอนต่อไป**: User ต้องรันสคริปต์ด้วย sudo แล้ว reboot (ถ้าจำเป็น)

### 2025-10-14 (Session 2)
⚠️ **Debug Boot Configuration**
- พบว่า FDT line อยู่ใน label ที่ไม่ได้ boot
- ตรวจสอบ merged DTB ว่ามี IMX219 nodes ครบถ้วน
- วางแผนวิธีแก้ไข

### 2025-10-14 (Session 1)
✅ **Calibration scripts พร้อมใช้งาน**
- สร้าง capture, calibrate, และ verify scripts เรียบร้อย
- เอกสารครบถ้วน (README.md, CALIBRATION_GUIDE.md)
- Chessboard pattern 7x7 inner corners พร้อมใช้งาน

### 2025-10-08
✅ **กล้อง stereo ทั้งสองตัวทำงานได้สมบูรณ์**
- Device tree overlay config สำเร็จ (ใช้ fdtoverlay merge DTB)
- ทดสอบกล้องทั้งสองตัวผ่าน (sensor-id=0 และ sensor-id=1)
- พร้อมสำหรับ calibration และ stereo vision applications
