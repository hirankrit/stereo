#!/usr/bin/env python3
"""
Test script for stereo camera on Jetson Orin Nano
"""
import cv2


def gstreamer_pipeline(
    sensor_id=0,
    capture_width=1920,
    capture_height=1080,
    display_width=960,
    display_height=540,
    framerate=30,
    flip_method=0,
):
    """
    GStreamer pipeline for Jetson CSI camera
    """
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} ! "
        f"video/x-raw(memory:NVMM), width=(int){capture_width}, height=(int){capture_height}, "
        f"framerate=(fraction){framerate}/1 ! "
        f"nvvidconv flip-method={flip_method} ! "
        f"video/x-raw, width=(int){display_width}, height=(int){display_height}, format=(string)BGRx ! "
        f"videoconvert ! "
        f"video/x-raw, format=(string)BGR ! appsink"
    )


def main():
    # Open left camera (sensor-id=0)
    print("Opening left camera (sensor-id=0)...")
    cap_left = cv2.VideoCapture(gstreamer_pipeline(sensor_id=0), cv2.CAP_GSTREAMER)

    # Open right camera (sensor-id=1)
    print("Opening right camera (sensor-id=1)...")
    cap_right = cv2.VideoCapture(gstreamer_pipeline(sensor_id=1), cv2.CAP_GSTREAMER)

    # Check if cameras opened successfully
    if not cap_left.isOpened():
        print("ERROR: Unable to open left camera (sensor-id=0)")
        return
    else:
        print("✓ Left camera opened successfully")

    if not cap_right.isOpened():
        print("ERROR: Unable to open right camera (sensor-id=1)")
        print("Note: If you only have one camera, this is expected")
        cap_left.release()
        return
    else:
        print("✓ Right camera opened successfully")

    print("\nPress 'q' to quit")
    print("Press 's' to save snapshot")

    snapshot_count = 0

    while True:
        ret_left, frame_left = cap_left.read()
        ret_right, frame_right = cap_right.read()

        if ret_left and ret_right:
            # Display frames
            cv2.imshow('Left Camera', frame_left)
            cv2.imshow('Right Camera', frame_right)

            key = cv2.waitKey(1) & 0xFF

            # Quit on 'q'
            if key == ord('q'):
                break

            # Save snapshot on 's'
            elif key == ord('s'):
                cv2.imwrite(f'stereo_left_{snapshot_count}.jpg', frame_left)
                cv2.imwrite(f'stereo_right_{snapshot_count}.jpg', frame_right)
                print(f"Saved snapshot {snapshot_count}")
                snapshot_count += 1
        else:
            print("ERROR: Failed to read frames")
            break

    # Clean up
    cap_left.release()
    cap_right.release()
    cv2.destroyAllWindows()
    print("Camera test completed")


if __name__ == "__main__":
    main()
