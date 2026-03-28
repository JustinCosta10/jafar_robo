#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PointStamped


class GreenControlNode(Node):
    def __init__(self):
        super().__init__('green_control_node')
        self.latest_goal = None

        self.subscriber = self.create_subscription(
            PointStamped, "goal_point", self.goal_callback, 10
        )
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        self.timer = self.create_timer(0.1, self.control_loop)

        # Control parameters
        self.steering_kp = 1.5       # proportional gain for steering
        self.max_speed = 0.25        # max forward speed (m/s)
        self.min_speed = 0.08        # min forward speed when close (m/s)
        self.stop_distance = 0.25    # stop when closer than this (m)
        self.full_speed_distance = 2.0  # distance at which max speed is reached (m)
        self.goal_timeout = 1.0      # stop if no goal received for this long (s)

        self.get_logger().info("Green Control Node has started!")

    def goal_callback(self, msg):
        self.latest_goal = msg

    def control_loop(self):
        cmd = Twist()

        if self.latest_goal is None:
            self.publisher_.publish(cmd)
            return

        # Check if the goal is stale
        now = self.get_clock().now()
        goal_time = rclpy.time.Time.from_msg(self.latest_goal.header.stamp)
        age = (now - goal_time).nanoseconds / 1e9
        if age > self.goal_timeout:
            self.get_logger().info("Goal point stale, stopping.")
            self.publisher_.publish(cmd)
            return

        x = self.latest_goal.point.x  # lateral offset in meters (+ = right)
        z = self.latest_goal.point.z  # forward distance in meters

        # z == 0 means no target detected
        if z == 0.0:
            self.get_logger().info("No target detected, stopping.")
            self.publisher_.publish(cmd)
            return

        if z < self.stop_distance:
            self.get_logger().info(f"Target close ({z:.2f}m), stopping.")
            self.publisher_.publish(cmd)
            return

        # Proportional steering: steer toward the target (negative x = turn left)
        steer = -self.steering_kp * x
        cmd.angular.z = max(-1.0, min(1.0, steer))

        # Scale speed linearly with distance: slow near target, faster when far
        t = (z - self.stop_distance) / (self.full_speed_distance - self.stop_distance)
        t = max(0.0, min(1.0, t))
        cmd.linear.x = self.min_speed + t * (self.max_speed - self.min_speed)

        self.publisher_.publish(cmd)
        self.get_logger().debug(
            f"Driving: speed={cmd.linear.x:.2f}, steer={cmd.angular.z:.2f}, dist={z:.2f}m"
        )


def main(args=None):
    rclpy.init(args=args)
    node = GreenControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Green Control Node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
