#!/usr/bin/env python3
"""
Stereo Camera Calibration Script
Calibrates stereo camera setup using captured chessboard images

This script:
1. Loads calibration images from left and right cameras
2. Detects chessboard corners in each image
3. Calibrates individual cameras (intrinsic parameters)
4. Performs stereo calibration (extrinsic parameters)
5. Computes rectification and undistortion maps
6. Saves calibration results to file

Usage:
    python3 calibrate_stereo.py

Output:
    - stereo_calibration.npz: Calibration parameters
    - calibration_report.txt: Detailed calibration report
"""

import cv2
import numpy as np
import glob
import os
from datetime import datetime

# Chessboard parameters (must match capture settings)
CHESSBOARD_SIZE = (6, 6)  # Inner corners (7x7 squares = 6x6 inner corners)
SQUARE_SIZE = 25  # mm

# Input directory
INPUT_DIR = "calibration_images"

# Output files
CALIBRATION_FILE = "stereo_calibration.npz"
REPORT_FILE = "calibration_report.txt"


def load_images(directory, camera):
    """Load all images from a directory"""
    pattern = os.path.join(directory, camera, "*.jpg")
    images = glob.glob(pattern)
    images.sort()
    return images


def find_chessboard_corners(images, chessboard_size):
    """
    Find chessboard corners in all images
    Returns: object points, image points, and valid images
    """
    # Prepare object points (0,0,0), (1,0,0), (2,0,0) ... (6,6,0)
    objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)
    objp *= SQUARE_SIZE  # Scale by square size in mm

    # Arrays to store object points and image points
    objpoints = []  # 3D points in real world space
    imgpoints = []  # 2D points in image plane
    valid_images = []

    # Termination criteria for corner refinement
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    print(f"\nProcessing {len(images)} images...")

    for idx, fname in enumerate(images):
        img = cv2.imread(fname)
        if img is None:
            print(f"  [{idx+1}/{len(images)}] ✗ Failed to load: {os.path.basename(fname)}")
            continue

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Find chessboard corners
        ret, corners = cv2.findChessboardCorners(
            gray, chessboard_size,
            cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
        )

        if ret:
            # Refine corners
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

            objpoints.append(objp)
            imgpoints.append(corners)
            valid_images.append(fname)
            print(f"  [{idx+1}/{len(images)}] ✓ {os.path.basename(fname)}")
        else:
            print(f"  [{idx+1}/{len(images)}] ✗ Pattern not found: {os.path.basename(fname)}")

    return objpoints, imgpoints, valid_images, gray.shape[::-1]


def calibrate_camera(objpoints, imgpoints, image_size):
    """Calibrate individual camera"""
    print("\nCalibrating camera...")

    # Calibration flags
    flags = 0
    flags |= cv2.CALIB_FIX_ASPECT_RATIO
    flags |= cv2.CALIB_ZERO_TANGENT_DIST

    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, image_size, None, None, flags=flags
    )

    print(f"  RMS reprojection error: {ret:.4f} pixels")

    return ret, camera_matrix, dist_coeffs, rvecs, tvecs


def stereo_calibrate(objpoints, imgpoints_left, imgpoints_right,
                     camera_matrix_left, dist_coeffs_left,
                     camera_matrix_right, dist_coeffs_right,
                     image_size):
    """Perform stereo calibration"""
    print("\nPerforming stereo calibration...")

    # SHARED INTRINSICS APPROACH:
    # Both cameras are identical IMX219 sensors, so they should have
    # the same intrinsic parameters. Use LEFT camera params for BOTH.
    print("  Using SHARED INTRINSICS (left camera params for both)")
    print(f"    Left camera matrix: fx={camera_matrix_left[0,0]:.2f}, fy={camera_matrix_left[1,1]:.2f}")
    print(f"    Left distortion: {dist_coeffs_left.ravel()}")

    # Copy left camera parameters to right camera
    camera_matrix_right = camera_matrix_left.copy()
    dist_coeffs_right = dist_coeffs_left.copy()

    # Stereo calibration flags
    flags = 0
    flags |= cv2.CALIB_FIX_INTRINSIC      # Fix the intrinsic parameters
    flags |= cv2.CALIB_FIX_FOCAL_LENGTH    # Don't refine focal length
    flags |= cv2.CALIB_FIX_PRINCIPAL_POINT # Don't refine principal point

    # Termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-5)

    ret, camera_matrix_left, dist_coeffs_left, camera_matrix_right, dist_coeffs_right, R, T, E, F = cv2.stereoCalibrate(
        objpoints, imgpoints_left, imgpoints_right,
        camera_matrix_left, dist_coeffs_left,
        camera_matrix_right, dist_coeffs_right,
        image_size,
        criteria=criteria,
        flags=flags
    )

    print(f"  Stereo RMS error: {ret:.4f} pixels")

    return ret, R, T, E, F, camera_matrix_right, dist_coeffs_right


def stereo_rectify(camera_matrix_left, dist_coeffs_left,
                   camera_matrix_right, dist_coeffs_right,
                   image_size, R, T):
    """Compute rectification transforms"""
    print("\nComputing rectification...")

    R1, R2, P1, P2, Q, roi_left, roi_right = cv2.stereoRectify(
        camera_matrix_left, dist_coeffs_left,
        camera_matrix_right, dist_coeffs_right,
        image_size, R, T,
        flags=cv2.CALIB_ZERO_DISPARITY,
        alpha=0.9  # 0=crop to valid pixels, 1=keep all pixels
    )

    return R1, R2, P1, P2, Q, roi_left, roi_right


def compute_rectification_maps(camera_matrix_left, dist_coeffs_left,
                               camera_matrix_right, dist_coeffs_right,
                               image_size, R1, R2, P1, P2):
    """Compute undistortion and rectification maps"""
    print("\nComputing rectification maps...")

    map_left_x, map_left_y = cv2.initUndistortRectifyMap(
        camera_matrix_left, dist_coeffs_left, R1, P1,
        image_size, cv2.CV_32FC1
    )

    map_right_x, map_right_y = cv2.initUndistortRectifyMap(
        camera_matrix_right, dist_coeffs_right, R2, P2,
        image_size, cv2.CV_32FC1
    )

    return map_left_x, map_left_y, map_right_x, map_right_y


def save_calibration(filename, **params):
    """Save calibration parameters to file"""
    print(f"\nSaving calibration to {filename}...")
    np.savez(filename, **params)
    print("  ✓ Calibration saved successfully")


def generate_report(filename, **params):
    """Generate human-readable calibration report"""
    print(f"\nGenerating calibration report: {filename}...")

    with open(filename, 'w') as f:
        f.write("=" * 80 + "\n")
        f.write("STEREO CAMERA CALIBRATION REPORT\n")
        f.write("=" * 80 + "\n")
        f.write(f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
        f.write(f"Chessboard: {CHESSBOARD_SIZE[0]}x{CHESSBOARD_SIZE[1]} inner corners\n")
        f.write(f"Square size: {SQUARE_SIZE} mm\n")
        f.write("\n")

        # Calibration errors
        f.write("-" * 80 + "\n")
        f.write("CALIBRATION ERRORS\n")
        f.write("-" * 80 + "\n")
        f.write(f"Left camera RMS error:   {params['rms_left']:.4f} pixels\n")
        f.write(f"Right camera RMS error:  {params['rms_right']:.4f} pixels\n")
        f.write(f"Stereo RMS error:        {params['rms_stereo']:.4f} pixels\n")
        f.write("\n")

        # Image information
        f.write("-" * 80 + "\n")
        f.write("IMAGE INFORMATION\n")
        f.write("-" * 80 + "\n")
        f.write(f"Image size:              {params['image_size'][0]} x {params['image_size'][1]}\n")
        f.write(f"Valid image pairs:       {params['num_images']}\n")
        f.write("\n")

        # Left camera intrinsics
        f.write("-" * 80 + "\n")
        f.write("LEFT CAMERA INTRINSIC PARAMETERS\n")
        f.write("-" * 80 + "\n")
        f.write("Camera Matrix (K):\n")
        f.write(str(params['camera_matrix_left']) + "\n\n")
        f.write("Distortion Coefficients (k1, k2, p1, p2, k3):\n")
        f.write(str(params['dist_coeffs_left'].ravel()) + "\n\n")

        # Right camera intrinsics
        f.write("-" * 80 + "\n")
        f.write("RIGHT CAMERA INTRINSIC PARAMETERS\n")
        f.write("-" * 80 + "\n")
        f.write("Camera Matrix (K):\n")
        f.write(str(params['camera_matrix_right']) + "\n\n")
        f.write("Distortion Coefficients (k1, k2, p1, p2, k3):\n")
        f.write(str(params['dist_coeffs_right'].ravel()) + "\n\n")

        # Stereo parameters
        f.write("-" * 80 + "\n")
        f.write("STEREO EXTRINSIC PARAMETERS\n")
        f.write("-" * 80 + "\n")
        f.write("Rotation Matrix (R):\n")
        f.write(str(params['R']) + "\n\n")
        f.write("Translation Vector (T) [mm]:\n")
        f.write(str(params['T'].ravel()) + "\n\n")

        # Baseline
        baseline = np.linalg.norm(params['T'])
        f.write(f"Baseline (distance between cameras): {baseline:.2f} mm ({baseline/10:.2f} cm)\n\n")

        # Rectification
        f.write("-" * 80 + "\n")
        f.write("RECTIFICATION PARAMETERS\n")
        f.write("-" * 80 + "\n")
        f.write("Q Matrix (Disparity-to-Depth Mapping):\n")
        f.write(str(params['Q']) + "\n\n")

        f.write("=" * 80 + "\n")
        f.write("END OF REPORT\n")
        f.write("=" * 80 + "\n")

    print("  ✓ Report generated successfully")


def main():
    print("=" * 80)
    print("STEREO CAMERA CALIBRATION")
    print("=" * 80)

    # Check if input directory exists
    if not os.path.exists(INPUT_DIR):
        print(f"\n✗ Error: Input directory '{INPUT_DIR}' not found!")
        print("  Run 'python3 capture_calibration_images.py' first.")
        return

    # Load images
    print("\n1. Loading images...")
    left_images = load_images(INPUT_DIR, "left")
    right_images = load_images(INPUT_DIR, "right")

    print(f"  Found {len(left_images)} left images")
    print(f"  Found {len(right_images)} right images")

    if len(left_images) != len(right_images):
        print("\n✗ Error: Number of left and right images don't match!")
        return

    if len(left_images) < 10:
        print("\n✗ Error: Not enough images for calibration (need at least 10 pairs)")
        return

    # Find chessboard corners
    print("\n2. Detecting chessboard corners in LEFT images:")
    objpoints_left, imgpoints_left, valid_left, image_size = find_chessboard_corners(
        left_images, CHESSBOARD_SIZE
    )

    print("\n3. Detecting chessboard corners in RIGHT images:")
    objpoints_right, imgpoints_right, valid_right, _ = find_chessboard_corners(
        right_images, CHESSBOARD_SIZE
    )

    # Find common valid images
    valid_left_set = set([os.path.basename(f).replace('left_', '') for f in valid_left])
    valid_right_set = set([os.path.basename(f).replace('right_', '') for f in valid_right])
    common_valid = valid_left_set & valid_right_set

    print(f"\n✓ Valid image pairs: {len(common_valid)}")

    if len(common_valid) < 10:
        print("✗ Error: Not enough valid image pairs for calibration")
        return

    # Filter to keep only common valid pairs
    # Create dictionaries for faster lookup
    left_dict = {}
    right_dict = {}

    for i, fname in enumerate(valid_left):
        basename = os.path.basename(fname).replace('left_', '')
        left_dict[basename] = (objpoints_left[i], imgpoints_left[i])

    for i, fname in enumerate(valid_right):
        basename = os.path.basename(fname).replace('right_', '')
        right_dict[basename] = (objpoints_right[i], imgpoints_right[i])

    # Filter common pairs
    objpoints = []
    imgpoints_left_filtered = []
    imgpoints_right_filtered = []

    for basename in sorted(common_valid):
        if basename in left_dict and basename in right_dict:
            objp_left, imgp_left = left_dict[basename]
            objp_right, imgp_right = right_dict[basename]
            objpoints.append(objp_left)
            imgpoints_left_filtered.append(imgp_left)
            imgpoints_right_filtered.append(imgp_right)

    # Calibrate individual cameras
    print("\n" + "=" * 80)
    print("4. CALIBRATING LEFT CAMERA")
    print("=" * 80)
    rms_left, camera_matrix_left, dist_coeffs_left, rvecs_left, tvecs_left = calibrate_camera(
        objpoints, imgpoints_left_filtered, image_size
    )

    print("\n" + "=" * 80)
    print("5. CALIBRATING RIGHT CAMERA")
    print("=" * 80)
    rms_right, camera_matrix_right, dist_coeffs_right, rvecs_right, tvecs_right = calibrate_camera(
        objpoints, imgpoints_right_filtered, image_size
    )

    # Stereo calibration
    print("\n" + "=" * 80)
    print("6. STEREO CALIBRATION")
    print("=" * 80)
    rms_stereo, R, T, E, F, camera_matrix_right, dist_coeffs_right = stereo_calibrate(
        objpoints, imgpoints_left_filtered, imgpoints_right_filtered,
        camera_matrix_left, dist_coeffs_left,
        camera_matrix_right, dist_coeffs_right,
        image_size
    )

    # Stereo rectification
    print("\n" + "=" * 80)
    print("7. STEREO RECTIFICATION")
    print("=" * 80)
    R1, R2, P1, P2, Q, roi_left, roi_right = stereo_rectify(
        camera_matrix_left, dist_coeffs_left,
        camera_matrix_right, dist_coeffs_right,
        image_size, R, T
    )

    # Compute rectification maps
    map_left_x, map_left_y, map_right_x, map_right_y = compute_rectification_maps(
        camera_matrix_left, dist_coeffs_left,
        camera_matrix_right, dist_coeffs_right,
        image_size, R1, R2, P1, P2
    )

    # Save calibration
    print("\n" + "=" * 80)
    print("8. SAVING CALIBRATION")
    print("=" * 80)

    calibration_params = {
        'camera_matrix_left': camera_matrix_left,
        'dist_coeffs_left': dist_coeffs_left,
        'camera_matrix_right': camera_matrix_right,
        'dist_coeffs_right': dist_coeffs_right,
        'R': R,
        'T': T,
        'E': E,
        'F': F,
        'R1': R1,
        'R2': R2,
        'P1': P1,
        'P2': P2,
        'Q': Q,
        'roi_left': roi_left,
        'roi_right': roi_right,
        'map_left_x': map_left_x,
        'map_left_y': map_left_y,
        'map_right_x': map_right_x,
        'map_right_y': map_right_y,
        'image_size': image_size,
        'rms_left': rms_left,
        'rms_right': rms_right,
        'rms_stereo': rms_stereo,
        'num_images': len(objpoints),
        'chessboard_size': CHESSBOARD_SIZE,
        'square_size': SQUARE_SIZE
    }

    save_calibration(CALIBRATION_FILE, **calibration_params)
    generate_report(REPORT_FILE, **calibration_params)

    # Summary
    print("\n" + "=" * 80)
    print("CALIBRATION COMPLETED SUCCESSFULLY!")
    print("=" * 80)
    print("\nCalibration results:")
    print(f"  • Left camera RMS error:  {rms_left:.4f} pixels")
    print(f"  • Right camera RMS error: {rms_right:.4f} pixels")
    print(f"  • Stereo RMS error:       {rms_stereo:.4f} pixels")
    print(f"  • Valid image pairs used: {len(objpoints)}")
    print(f"  • Baseline:               {np.linalg.norm(T):.2f} mm")
    print("\nOutput files:")
    print(f"  • {CALIBRATION_FILE} - Calibration data")
    print(f"  • {REPORT_FILE} - Detailed report")
    print("\nNext steps:")
    print("  1. Review calibration report")
    print("  2. Run verification: python3 verify_calibration.py")
    print("  3. Test depth estimation: python3 depth_map.py")
    print("=" * 80)


if __name__ == "__main__":
    main()
