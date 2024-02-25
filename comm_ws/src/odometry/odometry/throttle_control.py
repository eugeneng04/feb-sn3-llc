#!/usr/bin/env python3
import rclpy
from rclpy.node import Node 
from std_msgs.msg import Int32
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3

from .controller.PID import PID


class subscriber(Node):
    
    def __init__(self):
        super().__init__("throttle_control")
        self.wss_subscriber = self.create_subscription(Int32, "odometry/wss", self.wss_callback, 10)
        self.imu_subscriber = self.create_subscription(Imu, "odometry/imu", self.imu_callback, 10)
        self.target_subscriber = self.create_subscription(Int32, "control/throttle", self.set_target, 10)

        self.mass = 308
        self.drive_ratio = 3.5
        self.max_torque = 230
        self.tire_radius = 2.2
        self.pid = PID(1, 0.1, 0.1, 12, 0.1)
        self.v = 0
        self.a = 0
        self.timer = self.create_timer(0.01, self.pid_callback)

    def wss_callback(self, msg: Int32):
        self.v = msg.data * self.tire_radius
        self.get_logger().info(f"received wheel speed: {str(msg)}")

    def imu_callback(self, msg: Imu):
        self.a = (((msg.linear_acceleration).x) ** 2) + ((msg.linear_acceleration.y) ** 2) ** 0.5
        self.get_logger().info(f"received  imu data: {str(msg)}")

    def set_target(self, msg: Int32):
        self.pid.setTarget(msg.data)

    def accelToTorque(self, accel):
        return min((accel * self.tire_radius * self.mass)/self.drive_ratio,self.max_torque)
    
    def pid_callback(self):
        signal = self.pid.compute(self.a)
        throttle = self.accelToTorque(signal)
        self.control_throttle(throttle)
    
    def control_throttle(self, throttle):
        #send CAN signal for throttle
        self.get_logger().info(f"sending throttle signal: {throttle}Nm")
    
def main(args = None):
    rclpy.init(args = args)

    node = subscriber()

    rclpy.spin(node)
    rclpy.shutdown()
