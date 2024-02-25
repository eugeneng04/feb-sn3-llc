#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from sensor_msgs.msg import Imu

class publisher(Node):

    def __init__(self):
        super().__init__("odometry")
        self.steer_publisher = self.create_publisher(Int32, "odometry/steer", 10) # topic "odometry/steer"
        self.wss_publisher = self.create_publisher(Int32, "odometry/wss", 10) # topic "odometry/wss"
        self.imu_publisher = self.create_publisher(Imu, "odometry/imu", 10) # topic "odometry/imu"

        self.test_publisher = self.create_publisher(Int32, "control/throttle", 10)
        self.get_logger().info("Odometry publisher node has been started...")
        self.timer = self.create_timer(1, self.publish)
        self.i = 0

    def publish(self):
        steerMsg, wssMsg, imuMsg = Int32(), Int32(), Imu()
        throttleMsg = Int32()
        throttleMsg.data = self.i
        self.i += 1
        steerMsg.data, wssMsg.data = 0, 1
        #imuMsg.angular_velocity, imuMsg.linear_acceleration, imuMsg.orientation = 0, 0, 0
        self.steer_publisher.publish(steerMsg)
        self.wss_publisher.publish(wssMsg)
        self.imu_publisher.publish(imuMsg)
        self.test_publisher.publish(throttleMsg)
        self.get_logger().info(f"Publishing steering message with message {steerMsg.data}")
        self.get_logger().info(f"Publishing wss message with message {wssMsg.data}")
        self.get_logger().info(f"Publishing imu message with message {str(imuMsg)}")


def main(args = None):
    rclpy.init(args = args)
    node = publisher()
    rclpy.spin(node) #run indef until killed by ctrl-c for callbacks, also enables all callbacks for node


    rclpy.shutdown()


if __name__ == "__main__":
    main()