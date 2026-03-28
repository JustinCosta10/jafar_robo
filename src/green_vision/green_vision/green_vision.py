import yaml
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import os
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
from realsense2_camera_msgs.msg import Extrinsics


class GreenVisionNode(Node):
    def __init__(self):
        super().__init__('green_vision')
        self.bridge = CvBridge()
        self.upper_hsv = None
        self.lower_hsv = None
        self.paper_width = None
        self.paper_height = None
        self.latest_depth_image = None
        self.latest_centroid = None

        # Color camera intrinsics
        self.color_fx = None
        self.color_fy = None
        self.color_cx = None
        self.color_cy = None
        self.has_color_info = False

        # Depth camera intrinsics
        self.depth_fx = None
        self.depth_fy = None
        self.depth_cx = None
        self.depth_cy = None
        self.has_depth_info = False

        # Depth-to-color extrinsics
        self.extrinsic_rotation = None    # 3x3 rotation matrix
        self.extrinsic_translation = None  # 3-element translation vector
        self.has_extrinsics = False

        self._setup_parameters()
        self._setup_subscribers()
        self._setup_publishers()

    def _setup_parameters(self):
        default_config = os.path.join(
            get_package_share_directory("green_vision"),
            "config",
            "rgb_config.yaml",
        )
        config_path = self.declare_parameter(
            "config_path",
            default_config
        ).get_parameter_value().string_value

        try:
            with open(config_path, "r") as f:
                self.config = yaml.safe_load(f)
        except FileNotFoundError:
            self.get_logger().fatal(
                f"Config file not found: '{config_path}'. "
                "Pass the absolute path via: --ros-args -p config_path:=/absolute/path/to/rgb_config.yaml"
            )
            raise
        config = self.config["paper_config"]
        self.lower_hsv = np.array(config["lower_hsv"])
        self.upper_hsv = np.array(config["upper_hsv"])
        self.paper_width = config["paper_width"]
        self.paper_height = config["paper_height"]

    def _setup_publishers(self):
        self.debug_pub = self.create_publisher(Image, "perception/rgb_debug", 10)
        self.point_pub = self.create_publisher(PointStamped, "goal_point", 10)

    def _setup_subscribers(self):
        self.image_sub = self.create_subscription(
            Image, "/camera/camera/color/image_raw", self.image_callback, 1
        )
        self.depth_sub = self.create_subscription(
            Image, "/camera/camera/depth/image_rect_raw", self.depth_callback, 1
        )
        self.color_info_sub = self.create_subscription(
            CameraInfo, "/camera/camera/color/camera_info", self.color_info_callback, 10
        )
        self.depth_info_sub = self.create_subscription(
            CameraInfo, "/camera/camera/depth/camera_info", self.depth_info_callback, 10
        )
        self.extrinsics_sub = self.create_subscription(
            Extrinsics, "/camera/camera/extrinsics/depth_to_color",
            self.extrinsics_callback, 10
        )

    def color_info_callback(self, msg):
        self.color_fx = msg.k[0]
        self.color_fy = msg.k[4]
        self.color_cx = msg.k[2]
        self.color_cy = msg.k[5]
        if not self.has_color_info:
            self.has_color_info = True
            self.get_logger().info(
                f"Got color intrinsics: fx={self.color_fx:.1f}, fy={self.color_fy:.1f}, "
                f"cx={self.color_cx:.1f}, cy={self.color_cy:.1f}"
            )

    def depth_info_callback(self, msg):
        self.depth_fx = msg.k[0]
        self.depth_fy = msg.k[4]
        self.depth_cx = msg.k[2]
        self.depth_cy = msg.k[5]
        if not self.has_depth_info:
            self.has_depth_info = True
            self.get_logger().info(
                f"Got depth intrinsics: fx={self.depth_fx:.1f}, fy={self.depth_fy:.1f}, "
                f"cx={self.depth_cx:.1f}, cy={self.depth_cy:.1f}"
            )

    def extrinsics_callback(self, msg):
        self.extrinsic_rotation = np.array(msg.rotation).reshape(3, 3)
        self.extrinsic_translation = np.array(msg.translation)
        if not self.has_extrinsics:
            self.has_extrinsics = True
            self.get_logger().info(
                f"Got depth-to-color extrinsics: t={self.extrinsic_translation}"
            )

    def depth_callback(self, msg):
        try:
            self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as e:
            self.get_logger().error(f"Failed to convert depth image: {e}")

    def image_callback(self, msg):
        try:
            # 1. Convert ROS Image to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # 2. Convert to HSV for robustness since lighting messes with RGB
            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

            # 3. Color mask
            mask = cv2.inRange(hsv_image, self.lower_hsv, self.upper_hsv)

            # 4. Find contours
            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )

            # 5. Compute centroid
            published = False
            if contours:
                max_contour = max(contours, key=cv2.contourArea)
                M = cv2.moments(max_contour)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    self.latest_centroid = (cX, cY)

                    # 6. Look up depth at centroid and publish
                    published = self._publish_point(cX, cY, msg.header)

                    # Draw centroid on debug image
                    cv2.circle(cv_image, (cX, cY), 5, (0, 255, 0), -1)

            # If we didn't publish a valid goal, signal no target
            if not published:
                no_target = PointStamped()
                no_target.header.stamp = msg.header.stamp
                no_target.header.frame_id = "camera_color_optical_frame"
                self.point_pub.publish(no_target)

            # 7. Publish debug image
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))

        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")

    def _get_aligned_depth(self, cX, cY, depth_image):
        """Look up depth at a color pixel by transforming to the depth image frame.

        Uses the extrinsics to map the color pixel into the depth image,
        with an initial guess from the depth image to avoid iterative search.
        Returns depth in meters in the color camera frame, or None.
        """
        h, w = depth_image.shape[:2]

        # Inverse of depth_to_color: color_to_depth
        R_inv = self.extrinsic_rotation.T
        t_inv = -R_inv @ self.extrinsic_translation

        # Get an initial depth estimate by scaling color pixel to depth resolution
        color_info = (self.color_cx * 2, self.color_cy * 2)  # approximate color resolution
        dx_init = int(cX * w / color_info[0])
        dy_init = int(cY * h / color_info[1])
        dx_init = min(max(dx_init, 0), w - 1)
        dy_init = min(max(dy_init, 0), h - 1)
        raw_init = float(depth_image[dy_init, dx_init])

        if raw_init <= 0 or np.isnan(raw_init):
            return None

        # Use the initial depth to project color pixel to 3D, transform, and refine
        z_est = raw_init / 1000.0
        ray_color = np.array([
            (cX - self.color_cx) / self.color_fx,
            (cY - self.color_cy) / self.color_fy,
            1.0,
        ])

        pt_color = ray_color * z_est
        pt_depth = R_inv @ pt_color + t_inv

        if pt_depth[2] <= 0:
            return None

        # Project into depth image
        dx = int(round(pt_depth[0] * self.depth_fx / pt_depth[2] + self.depth_cx))
        dy = int(round(pt_depth[1] * self.depth_fy / pt_depth[2] + self.depth_cy))

        if not (0 <= dx < w and 0 <= dy < h):
            return None

        raw = float(depth_image[dy, dx])
        if raw <= 0 or np.isnan(raw):
            return None

        # Convert depth reading back to color frame z
        # Scale the color ray so the depth-frame z matches the reading
        depth_z = raw / 1000.0
        scale = depth_z / pt_depth[2] * z_est
        return scale

    def _publish_point(self, cX, cY, header):
        if not self.has_color_info or not self.has_depth_info or not self.has_extrinsics:
            self.get_logger().warn("Waiting for camera info and extrinsics, skipping point publish.")
            return False

        if self.latest_depth_image is None:
            self.get_logger().warn("No depth image received yet, skipping point publish.")
            return False

        depth_image = self.latest_depth_image

        z = self._get_aligned_depth(cX, cY, depth_image)
        if z is None:
            self.get_logger().warn(f"Could not align depth at ({cX}, {cY}), skipping.")
            return False

        # Project to 3D in color camera frame using color intrinsics
        x = (cX - self.color_cx) * z / self.color_fx
        y = (cY - self.color_cy) * z / self.color_fy

        point_msg = PointStamped()
        point_msg.header.stamp = header.stamp
        point_msg.header.frame_id = "camera_color_optical_frame"
        point_msg.point.x = x
        point_msg.point.y = y
        point_msg.point.z = z

        self.point_pub.publish(point_msg)
        self.get_logger().info(f"Published point: x={x:.3f}m, y={y:.3f}m, z={z:.3f}m")
        return True


def main(args=None):
    rclpy.init(args=args)
    node = GreenVisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Green Vision Node...")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
