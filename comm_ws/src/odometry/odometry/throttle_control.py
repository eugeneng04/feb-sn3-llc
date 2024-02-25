#!/usr/bin/env python3
import rclpy
from rclpy.node import Node 
from std_msgs.msg import Int32

class subscriber(Node):
    
    def __init__(self):
        super().__init__("subscriber")
        self.wss_subscriber = self.create_subscription(Int32, "odometry/wss", self.wss_callback, 10)

    def wss_callback(self, msg: Int32):
        self.get_logger().info(f"received wheel speed: {str(msg)}")

def main(args = None):
    rclpy.init(args = args)

    node = subscriber()

    rclpy.spin(node)
    rclpy.shutdown()
