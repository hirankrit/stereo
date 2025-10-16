#!/usr/bin/env python3
"""
Check calibration image quality - brightness analysis
"""

import cv2
import numpy as np
import os
import glob

def check_images(folder):
    """Check image brightness statistics"""
    images = sorted(glob.glob(os.path.join(folder, "*.jpg")))

    if not images:
        print(f"No images found in {folder}")
        return None

    print(f"\n{folder}:")
    print(f"Total images: {len(images)}")

    mean_values = []
    max_values = []
    min_values = []

    for img_path in images:
        img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
        if img is not None:
            mean_values.append(np.mean(img))
            max_values.append(np.max(img))
            min_values.append(np.min(img))

    if mean_values:
        print(f"Mean intensity: {np.mean(mean_values):.1f} (min: {np.min(mean_values):.1f}, max: {np.max(mean_values):.1f})")
        print(f"Max intensity: {np.mean(max_values):.1f}")
        print(f"Min intensity: {np.mean(min_values):.1f}")

        # Quality assessment
        avg_mean = np.mean(mean_values)
        if avg_mean > 120:
            print("✅ Quality: EXCELLENT - Very bright")
        elif avg_mean > 100:
            print("✅ Quality: GOOD - Bright enough")
        elif avg_mean > 80:
            print("⚠️  Quality: ACCEPTABLE - Could be brighter")
        elif avg_mean > 60:
            print("⚠️  Quality: MARGINAL - May affect calibration")
        else:
            print("❌ Quality: POOR - Too dark")

        return {
            'mean': np.mean(mean_values),
            'max': np.mean(max_values),
            'min': np.mean(min_values)
        }

    return None

if __name__ == "__main__":
    print("=" * 60)
    print("CALIBRATION IMAGE QUALITY CHECK")
    print("=" * 60)

    left_stats = check_images("calibration_images/left")
    right_stats = check_images("calibration_images/right")

    if left_stats and right_stats:
        print("\n" + "=" * 60)
        print("COMPARISON WITH PREVIOUS ROUND")
        print("=" * 60)
        print("Previous round (Session 7):")
        print("  Left mean: 47.0, Right mean: 53.0")
        print(f"\nCurrent round:")
        print(f"  Left mean: {left_stats['mean']:.1f}, Right mean: {right_stats['mean']:.1f}")

        improvement_left = left_stats['mean'] - 47.0
        improvement_right = right_stats['mean'] - 53.0

        print(f"\nImprovement:")
        print(f"  Left: {improvement_left:+.1f} ({improvement_left/47.0*100:+.1f}%)")
        print(f"  Right: {improvement_right:+.1f} ({improvement_right/53.0*100:+.1f}%)")

        if left_stats['mean'] > 100 and right_stats['mean'] > 100:
            print("\n✅ READY FOR CALIBRATION - Image quality is good!")
        elif left_stats['mean'] > 80 and right_stats['mean'] > 80:
            print("\n⚠️  ACCEPTABLE - You can try calibration, but results may vary")
        else:
            print("\n❌ NOT RECOMMENDED - Consider increasing lighting or gain")
