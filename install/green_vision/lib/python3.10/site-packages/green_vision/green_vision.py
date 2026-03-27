import yaml
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import os
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory

class GreenVisionNode(Node):
    def __init__(self):
        super().__init__('green_vision')
        self.bridge = CvBridge()
        self.upper_hsv = None
        self.lower_hsv = None
        self.hsv = None
        self.rgb = None
        self.paper_width = None
        self.paper_height = None
        self.x = None
        self.y = None
        self.z = None

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
            Image, "/camera/color/image_raw", self.image_callback, 10
        )
        self.depth_sub = self.create_subscription(Image, "/camera/depth/image_rect_raw", depth_callback)
    def depth_callback(self, msg):
        pass

        
    def image_callback(self, msg):

        try:
            # 1. Convert ROS Image to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # 2. convert img to HSV for robustness since lighting messes w RGB
            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

            # 3. color mask
            mask = cv2.inRange(hsv_image, self.lower_hsv, self.upper_hsv)

            # 4. find contours
            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )  # detected boundaries of masked areas
            #5. compute centroid
            if contours:
                max_countour = max(contours, key=cv2.contourArea)
                #compares the contours areas to find the largest area found
                M = cv2.moments(max_contour)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
            #6 create and publish message
                    point_msg = PointStamped()
                    
                    point_msg.point.x = cX
                    point_msg.point.y = cY
                    #self.x = cX
                    #self.y = cY
                    point_pub.publish(point_msg)
            else:
                node.get_logger().info("Mask not created.")

            # 7. draw debug info
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))

        except Exception as e:
            self.get_logger().error("Failed to process image: {e}")
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

