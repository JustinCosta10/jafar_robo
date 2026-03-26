"""
RPLIDAR depth perception for autonomous detection of wall

sub to LIDAR sensor /scan topic -> publish distance from wall
"""

import yaml
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32


class DepthNode(Node):
    def __init__(self):
        super().__init__("depth_node")

        self._setup_parameters()
        self._setup_subscriptions()
        self._setup_publishers()

    def _setup_parameters(self):
        pass

    def _setup_publishers(self):
        self.obstacle_dist_pub = self.create_publisher(
            Float32, "/perception/front_distance", 10
        )

    def _setup_subscriptions(self):
        """lidar depth sub"""
        self.depth_sub = self.create_subscription(
            LaserScan, "/scan", self.depth_callback, 10
        )

    def depth_callback(self, msg):
        dist_to_front_obstacle = msg.ranges[0]  # angle 0º => directly in front of rover
        # TODO may need to change if lidar is mounted backwards? (to 180 degs)

        dist_msg = Float32()
        dist_msg.data = dist_to_front_obstacle
        self.obstacle_dist_pub.publish(dist_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DepthNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
