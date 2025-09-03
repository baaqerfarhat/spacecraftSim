"""
ROS Controller Node - Quickstart
--------------------------------
This file gives you two options:
  1) Run the built-in IMU/TF-based PID controller (default)
  2) Start from a minimal sample controller skeleton and build your own

Topics you likely need:
  - IMU in:      /srb/env0/imu_robot            (sensor_msgs/Imu)
  - Camera in:   /srb/env0/robot/camera/image_raw (sensor_msgs/Image)
  - Thrust cmd:  /srb/env0/robot/thrust         (std_msgs/Float32MultiArray)
                  data length = 8 (thrusters), values in [0..1]

Switch controllers:
  python3 ros_controller_node.py --controller imu     # built-in controller
  python3 ros_controller_node.py --controller sample  # your own skeleton
"""

import argparse
import time
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Imu, Image
from std_msgs.msg import Float32MultiArray

# Use the existing controller implementation
from space_robotics_bench.imu_pid_controller import IMUPIDController


class CustomController(Node):
    """Minimal skeleton controller you can extend.

    Subscribes:
      - /srb/env0/imu_robot (sensor_msgs/Imu)
      - /srb/env0/robot/camera/image_raw (sensor_msgs/Image)

    Publishes:
      - /srb/env0/robot/thrust (std_msgs/Float32MultiArray)
    """

    def __init__(self,
                 imu_topic: str = '/srb/env0/imu_robot',
                 image_topic: Optional[str] = '/srb/env0/robot/camera/image_raw',
                 thrust_topic: str = '/srb/env0/robot/thrust',
                 rate_hz: float = 20.0):
        super().__init__('custom_spacecraft_controller')

        # I/O
        self.thrust_pub = self.create_publisher(Float32MultiArray, thrust_topic, 10)
        self.imu_sub = self.create_subscription(Imu, imu_topic, self._imu_cb, 10)
        self.image_sub = None
        if image_topic:
            self.image_sub = self.create_subscription(Image, image_topic, self._img_cb, 10)

        # State buffers
        self.last_imu = None
        self.last_image = None
        self.last_imu_time = 0.0
        self.last_image_time = 0.0

        # Control loop
        period = 1.0 / max(rate_hz, 1.0)
        self.timer = self.create_timer(period, self._control_loop)
        self.get_logger().info('CustomController started. Edit _control_loop to implement your policy.')

    def _imu_cb(self, msg: Imu) -> None:
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9 or time.time()
        self.last_imu = msg
        self.last_imu_time = t

    def _img_cb(self, msg: Image) -> None:
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9 or time.time()
        self.last_image = msg
        self.last_image_time = t

    def _control_loop(self) -> None:
        # TODO: replace this with your controller logic
        # Example: zero thrust (stop)
        thrust = Float32MultiArray()
        thrust.data = [0.0] * 8
        self.thrust_pub.publish(thrust)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--controller', choices=['imu', 'sample'], default='imu')
    parser.add_argument('--imu-topic', default='/srb/env0/imu_robot')
    parser.add_argument('--image-topic', default='/srb/env0/robot/camera/image_raw')
    parser.add_argument('--thrust-topic', default='/srb/env0/robot/thrust')
    parser.add_argument('--rate-hz', type=float, default=20.0)
    args = parser.parse_args()

    rclpy.init()

    if args.controller == 'sample':
        node = CustomController(
            imu_topic=args.imu_topic,
            image_topic=args.image_topic,
            thrust_topic=args.thrust_topic,
            rate_hz=args.rate_hz,
        )
    else:
        node = IMUPIDController()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if isinstance(node, IMUPIDController):
            node.stop_pid_control()
        rclpy.shutdown()


if __name__ == "__main__":
    main()


