#!/usr/bin/env python3
"""
Stereo Camera Node for NVIDIA Jetson IMX219
Uses nvarguscamerasrc for native CSI camera support
Publishes synchronized stereo image pairs
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np


class StereoCameraNode(Node):
    def __init__(self):
        super().__init__('stereo_camera_node')

        # Declare parameters
        self.declare_parameter('left_sensor_id', 0)
        self.declare_parameter('right_sensor_id', 1)
        self.declare_parameter('width', 1280)
        self.declare_parameter('height', 720)
        self.declare_parameter('framerate', 30)
        self.declare_parameter('flip_method', 0)

        # Get parameters
        self.left_sensor_id = self.get_parameter('left_sensor_id').value
        self.right_sensor_id = self.get_parameter('right_sensor_id').value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.framerate = self.get_parameter('framerate').value
        self.flip_method = self.get_parameter('flip_method').value

        # Create publishers
        self.left_image_pub = self.create_publisher(
            Image, '/stereo/left/image_raw', 10)
        self.right_image_pub = self.create_publisher(
            Image, '/stereo/right/image_raw', 10)
        self.left_info_pub = self.create_publisher(
            CameraInfo, '/stereo/left/camera_info', 10)
        self.right_info_pub = self.create_publisher(
            CameraInfo, '/stereo/right/camera_info', 10)

        # CV Bridge
        self.bridge = CvBridge()

        # Build GStreamer pipelines
        self.left_pipeline = self.build_gstreamer_pipeline(
            self.left_sensor_id)
        self.right_pipeline = self.build_gstreamer_pipeline(
            self.right_sensor_id)

        self.get_logger().info(
            f'Left pipeline: {self.left_pipeline}')
        self.get_logger().info(
            f'Right pipeline: {self.right_pipeline}')

        # Open cameras (with delay to avoid resource conflict)
        import time

        self.get_logger().info('Opening left camera...')
        self.left_cap = cv2.VideoCapture(
            self.left_pipeline, cv2.CAP_GSTREAMER)

        # Wait for left camera to fully initialize
        time.sleep(2)

        self.get_logger().info('Opening right camera...')
        self.right_cap = cv2.VideoCapture(
            self.right_pipeline, cv2.CAP_GSTREAMER)

        if not self.left_cap.isOpened():
            self.get_logger().error(
                f'Failed to open left camera (sensor_id={self.left_sensor_id})')
            raise RuntimeError('Left camera failed to open')

        if not self.right_cap.isOpened():
            self.get_logger().error(
                f'Failed to open right camera (sensor_id={self.right_sensor_id})')
            raise RuntimeError('Right camera failed to open')

        self.get_logger().info('Both cameras opened successfully!')

        # Create timer for capturing frames
        self.timer = self.create_timer(
            1.0 / self.framerate, self.timer_callback)

        # Frame counter
        self.frame_count = 0

    def build_gstreamer_pipeline(self, sensor_id):
        """
        Build GStreamer pipeline for nvarguscamerasrc

        Args:
            sensor_id: Camera sensor ID (0 or 1)

        Returns:
            GStreamer pipeline string
        """
        return (
            f'nvarguscamerasrc sensor-id={sensor_id} ! '
            f'video/x-raw(memory:NVMM), '
            f'width=(int){self.width}, height=(int){self.height}, '
            f'format=(string)NV12, framerate=(fraction){self.framerate}/1 ! '
            f'nvvidconv flip-method={self.flip_method} ! '
            f'video/x-raw, format=(string)BGRx ! '
            f'videoconvert ! '
            f'video/x-raw, format=(string)BGR ! '
            f'appsink'
        )

    def timer_callback(self):
        """
        Capture and publish stereo camera frames
        """
        # Read frames from both cameras
        ret_left, frame_left = self.left_cap.read()
        ret_right, frame_right = self.right_cap.read()

        if not ret_left or not ret_right:
            self.get_logger().warn(
                f'Failed to capture frames: '
                f'left={ret_left}, right={ret_right}')
            return

        # Get timestamp (same for both cameras)
        timestamp = self.get_clock().now().to_msg()

        # Publish images
        try:
            # Left image
            left_msg = self.bridge.cv2_to_imgmsg(frame_left, encoding='bgr8')
            left_msg.header.stamp = timestamp
            left_msg.header.frame_id = 'left_camera_optical_frame'
            self.left_image_pub.publish(left_msg)

            # Right image
            right_msg = self.bridge.cv2_to_imgmsg(
                frame_right, encoding='bgr8')
            right_msg.header.stamp = timestamp
            right_msg.header.frame_id = 'right_camera_optical_frame'
            self.right_image_pub.publish(right_msg)

        except Exception as e:
            self.get_logger().error(f'Failed to publish images: {e}')
            return

        # Publish camera info (basic - will need calibration later)
        left_info = CameraInfo()
        left_info.header.stamp = timestamp
        left_info.header.frame_id = 'left_camera_optical_frame'
        left_info.width = self.width
        left_info.height = self.height
        self.left_info_pub.publish(left_info)

        right_info = CameraInfo()
        right_info.header.stamp = timestamp
        right_info.header.frame_id = 'right_camera_optical_frame'
        right_info.width = self.width
        right_info.height = self.height
        self.right_info_pub.publish(right_info)

        self.frame_count += 1

        # Log every 30 frames
        if self.frame_count % 30 == 0:
            self.get_logger().info(
                f'Published {self.frame_count} stereo frame pairs')

    def destroy_node(self):
        """
        Cleanup on shutdown
        """
        self.get_logger().info('Shutting down stereo camera node...')
        if hasattr(self, 'left_cap') and self.left_cap is not None:
            self.left_cap.release()
        if hasattr(self, 'right_cap') and self.right_cap is not None:
            self.right_cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    try:
        node = StereoCameraNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
