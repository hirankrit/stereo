#!/usr/bin/env python3
"""
Stereo Calibration Verification Script
Visualizes the quality of stereo camera calibration

This script:
1. Loads saved calibration parameters
2. Opens stereo cameras
3. Applies rectification to live video
4. Draws horizontal epipolar lines for verification
5. Shows undistorted and rectified views

Good calibration indicators:
- Horizontal lines should pass through same points in both images
- Objects should appear at same vertical position
- Images should be aligned horizontally

Usage:
    python3 verify_calibration.py

Controls:
    'l' - Toggle epipolar lines
    's' - Save verification image
    'q' or ESC - Quit
"""

import cv2
import numpy as np
import os

# Camera parameters (must match calibration settings)
CAPTURE_WIDTH = 1280
CAPTURE_HEIGHT = 720
DISPLAY_WIDTH = 640
DISPLAY_HEIGHT = 360
FRAMERATE = 60
FLIP_METHOD = 0

# Calibration file
CALIBRATION_FILE = "stereo_calibration.npz"


def gstreamer_pipeline(
    sensor_id=0,
    capture_width=CAPTURE_WIDTH,
    capture_height=CAPTURE_HEIGHT,
    display_width=DISPLAY_WIDTH,
    display_height=DISPLAY_HEIGHT,
    framerate=FRAMERATE,
    flip_method=FLIP_METHOD,
):
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} "
        f"aelock=true awblock=true "
        f"exposuretimerange=\"33333333 33333333\" "
        f"gainrange=\"10 10\" ! "
        f"video/x-raw(memory:NVMM), width=(int){capture_width}, height=(int){capture_height}, "
        f"format=(string)NV12, framerate=(fraction){framerate}/1 ! "
        f"nvvidconv flip-method={flip_method} ! "
        f"video/x-raw, width=(int){display_width}, height=(int){display_height}, format=(string)BGRx ! "
        f"videoconvert ! "
        f"video/x-raw, format=(string)BGR ! appsink"
    )


def draw_epipolar_lines(img_left, img_right, num_lines=20):
    """Draw horizontal epipolar lines on both images"""
    height = img_left.shape[0]
    step = height // (num_lines + 1)

    for i in range(1, num_lines + 1):
        y = i * step
        color = (0, 255, 0) if i % 2 == 0 else (0, 255, 255)
        cv2.line(img_left, (0, y), (img_left.shape[1], y), color, 1)
        cv2.line(img_right, (0, y), (img_right.shape[1], y), color, 1)

    return img_left, img_right


def load_calibration(filename):
    """Load calibration parameters from file"""
    if not os.path.exists(filename):
        print(f"✗ Error: Calibration file '{filename}' not found!")
        print("  Run 'python3 calibrate_stereo.py' first.")
        return None

    print(f"Loading calibration from {filename}...")
    data = np.load(filename)

    calibration = {
        'map_left_x': data['map_left_x'],
        'map_left_y': data['map_left_y'],
        'map_right_x': data['map_right_x'],
        'map_right_y': data['map_right_y'],
        'Q': data['Q'],
        'roi_left': data['roi_left'],
        'roi_right': data['roi_right'],
        'image_size': data['image_size'],
        'rms_stereo': data['rms_stereo'],
    }

    print("  ✓ Calibration loaded successfully")
    print(f"  • Image size: {calibration['image_size'][0]} x {calibration['image_size'][1]}")
    print(f"  • Stereo RMS error: {calibration['rms_stereo']:.4f} pixels")

    return calibration


def main():
    print("=" * 80)
    print("STEREO CALIBRATION VERIFICATION")
    print("=" * 80)
    print("\nThis tool helps verify the quality of your stereo calibration.")
    print("\nGood calibration indicators:")
    print("  • Horizontal lines pass through same points in both images")
    print("  • Objects appear at same vertical position in both images")
    print("  • Images are properly aligned horizontally")
    print("\nControls:")
    print("  'l' - Toggle epipolar lines on/off")
    print("  's' - Save verification image")
    print("  'q' or ESC - Quit")
    print("=" * 80)

    # Load calibration
    calibration = load_calibration(CALIBRATION_FILE)
    if calibration is None:
        return

    # Open cameras
    print("\nOpening cameras...")
    cap_left = cv2.VideoCapture(gstreamer_pipeline(sensor_id=0), cv2.CAP_GSTREAMER)
    cap_right = cv2.VideoCapture(gstreamer_pipeline(sensor_id=1), cv2.CAP_GSTREAMER)

    if not cap_left.isOpened() or not cap_right.isOpened():
        print("✗ Error: Cannot open one or both cameras")
        return

    print("✓ Cameras opened successfully!")
    print("\nShowing rectified stereo view with epipolar lines...")
    print("Press 'l' to toggle lines, 's' to save, 'q' to quit\n")

    show_lines = True
    frame_count = 0

    while True:
        ret_left, frame_left = cap_left.read()
        ret_right, frame_right = cap_right.read()

        if not ret_left or not ret_right:
            print("✗ Error: Cannot read from cameras")
            break

        # Apply rectification
        rectified_left = cv2.remap(
            frame_left,
            calibration['map_left_x'],
            calibration['map_left_y'],
            cv2.INTER_LINEAR
        )
        rectified_right = cv2.remap(
            frame_right,
            calibration['map_right_x'],
            calibration['map_right_y'],
            cv2.INTER_LINEAR
        )

        # Create display copies
        display_left = rectified_left.copy()
        display_right = rectified_right.copy()

        # Draw epipolar lines if enabled
        if show_lines:
            display_left, display_right = draw_epipolar_lines(display_left, display_right)

        # Add status text
        status = "Lines: ON" if show_lines else "Lines: OFF"
        cv2.putText(display_left, status, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(display_right, status, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        cv2.putText(display_left, "Left Camera", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(display_right, "Right Camera", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        # Combine images side by side
        combined = np.hstack((display_left, display_right))

        # Add title
        title_bar = np.zeros((40, combined.shape[1], 3), dtype=np.uint8)
        cv2.putText(title_bar, "Rectified Stereo View - Press 'l' for lines, 's' to save, 'q' to quit",
                    (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        combined = np.vstack((title_bar, combined))

        cv2.imshow("Stereo Calibration Verification", combined)

        key = cv2.waitKey(1) & 0xFF

        # Toggle lines
        if key == ord('l'):
            show_lines = not show_lines
            print(f"Epipolar lines: {'ON' if show_lines else 'OFF'}")

        # Save image
        elif key == ord('s'):
            filename = f"verification_{frame_count:04d}.jpg"
            cv2.imwrite(filename, combined)
            print(f"✓ Saved: {filename}")
            frame_count += 1

        # Quit
        elif key == ord('q') or key == 27:  # ESC
            break

    # Cleanup
    cap_left.release()
    cap_right.release()
    cv2.destroyAllWindows()

    print("\n" + "=" * 80)
    print("Verification completed")
    print("=" * 80)
    print("\nCalibration quality assessment:")
    print(f"  • Stereo RMS error: {calibration['rms_stereo']:.4f} pixels")

    if calibration['rms_stereo'] < 0.5:
        print("  • Quality: ★★★★★ EXCELLENT - Ready for production use")
    elif calibration['rms_stereo'] < 1.0:
        print("  • Quality: ★★★★☆ VERY GOOD - Suitable for most applications")
    elif calibration['rms_stereo'] < 2.0:
        print("  • Quality: ★★★☆☆ GOOD - Acceptable for general use")
    elif calibration['rms_stereo'] < 3.0:
        print("  • Quality: ★★☆☆☆ FAIR - Consider recalibrating for better accuracy")
    else:
        print("  • Quality: ★☆☆☆☆ POOR - Recalibration recommended")

    print("\nIf the horizontal lines didn't align well:")
    print("  • Capture more calibration images (30+ pairs recommended)")
    print("  • Ensure chessboard is flat and well-lit")
    print("  • Cover all areas of the image during capture")
    print("  • Keep cameras stable (avoid vibration)")
    print("=" * 80)


if __name__ == "__main__":
    main()
