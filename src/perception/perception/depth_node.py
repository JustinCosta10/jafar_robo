"""
RPLIDAR depth perception for autonomous detection of wall

sub to LIDAR sensor /scan topic -> publish distance from wall

TODO: rover controller node should sub to this node to inform wall following behavior
"""

import yaml
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class DepthNode(Node):
    def __init__(self):
        super().__init__("depth_node")

        self._setup_parameters()
        self._setup_subscriptions()
        self._setup_publishers()

    def _setup_parameters(self):
        pass

    def _setup_publishers(self):
        pass

    def _setup_subscriptions(self):
        pass

    def depth_callback(self, msg):
        pass
