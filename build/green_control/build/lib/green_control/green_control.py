#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
# Example: Import a message type (like Twist for movement)
# from geometry_msgs.msg import Twist 

class GreenControlNode(Node):
    def __init__(self):
        # Initialize the node with the name 'green_control_node'
        super().__init__('green_control_node')
        self.subscriber = self.create_subscriber(PointStamped, "goal_point", 10)
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # 2. Setup a Timer (runs at 10Hz / every 0.1s)
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info("Green Control Node has started!")

    def control_loop(self):
        """This function runs repeatedly based on the timer."""
        # Logic for your control goes here
        msg = Twist()
        msg.linear.x = 0.5
        self.publisher_.publish(msg)
        
        self.get_logger().info("Running control loop...")

def main(args=None):
    # Initialize ROS 2 Python communications
    rclpy.init(args=args)
    
    # Create the node instance
    node = GreenControlNode()
    
    try:
        # Spin the node so callbacks (like the timer) can run
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Handle Ctrl+C gracefully
        node.get_logger().info("Shutting down Green Control Node...")
    finally:
        # Cleanup
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

