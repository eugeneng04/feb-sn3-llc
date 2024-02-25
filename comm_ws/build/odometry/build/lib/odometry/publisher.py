#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class publisher(Node):

    def __init__(self):
        super().__init__("odometry")
        self.steer_publisher = self.create_publisher(Int32, "odometry/steer", 10) # topic "odometry/steer"
        self.wss_publisher = self.create_publisher(Int32, "odometry/wss", 10) # topic "odometry/steer"
        self.get_logger().info("Odometry publisher node has been started...")
        self.timer = self.create_timer(1, self.publish)

    def publish(self):
        steerMsg, wssMsg = Int32(), Int32()
        steerMsg.data, wssMsg.data = 0, 1
        self.steer_publisher.publish(steerMsg)
        self.wss_publisher.publish(wssMsg)
        self.get_logger().info(f"Publishing steering message with message {steerMsg.data}")
        self.get_logger().info(f"Publishing wss message with message {wssMsg.data}")


def main(args = None):
    rclpy.init(args = args)
    node = publisher()
    rclpy.spin(node) #run indef until killed by ctrl-c for callbacks, also enables all callbacks for node


    rclpy.shutdown()


if __name__ == "__main__":
    main()