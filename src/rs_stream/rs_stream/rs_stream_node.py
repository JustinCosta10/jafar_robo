#!/usr/bin/env python3
"""
RealSense stream node — publishes aligned color and depth images.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import pyrealsense2 as rs
import numpy as np


class RsStreamNode(Node):
    def __init__(self, width=640, height=480, fps=30):
        super().__init__("rs_stream_node")

        self.bridge = CvBridge()

        # Publishers
        self.color_pub = self.create_publisher(Image, "/camera/color/image_raw", 10)
        self.depth_pub = self.create_publisher(Image, "/camera/depth/image_raw", 10)
        self.color_info_pub = self.create_publisher(CameraInfo, "/camera/color/camera_info", 10)

        # RealSense pipeline with alignment
        self.pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        cfg.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        self.get_logger().info("Starting RealSense pipeline...")
        profile = self.pipe.start(cfg)

        self.align = rs.align(rs.stream.color)

        # Build CameraInfo from color intrinsics
        color_profile = profile.get_stream(rs.stream.color).as_video_stream_profile()
        intr = color_profile.get_intrinsics()
        self.color_info_msg = CameraInfo()
        self.color_info_msg.header.frame_id = "camera_color_optical_frame"
        self.color_info_msg.width = intr.width
        self.color_info_msg.height = intr.height
        self.color_info_msg.distortion_model = "plumb_bob"
        self.color_info_msg.d = [float(c) for c in intr.coeffs]
        self.color_info_msg.k = [
            intr.fx, 0.0, intr.ppx,
            0.0, intr.fy, intr.ppy,
            0.0, 0.0, 1.0,
        ]
        self.color_info_msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        self.color_info_msg.p = [
            intr.fx, 0.0, intr.ppx, 0.0,
            0.0, intr.fy, intr.ppy, 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]

        self.timer = self.create_timer(1.0 / fps, self.capture_and_publish)

    def capture_and_publish(self):
        try:
            frames = self.pipe.wait_for_frames(timeout_ms=1000)
            aligned = self.align.process(frames)

            color_frame = aligned.get_color_frame()
            depth_frame = aligned.get_depth_frame()

            stamp = self.get_clock().now().to_msg()

            if color_frame:
                color_img = np.asanyarray(color_frame.get_data())
                color_msg = self.bridge.cv2_to_imgmsg(color_img, encoding="bgr8")
                color_msg.header.stamp = stamp
                color_msg.header.frame_id = "camera_color_optical_frame"
                self.color_pub.publish(color_msg)

                self.color_info_msg.header.stamp = stamp
                self.color_info_pub.publish(self.color_info_msg)

            if depth_frame:
                depth_img = np.asanyarray(depth_frame.get_data())
                depth_msg = self.bridge.cv2_to_imgmsg(depth_img, encoding="16UC1")
                depth_msg.header.stamp = stamp
                depth_msg.header.frame_id = "camera_depth_optical_frame"
                self.depth_pub.publish(depth_msg)

        except Exception as e:
            self.get_logger().warn(f"Frame skip: {e}")

    def destroy_node(self):
        try:
            self.pipe.stop()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RsStreamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
