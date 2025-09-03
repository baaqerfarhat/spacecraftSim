#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu

class VINSRelay(Node):
    def __init__(self):
        super().__init__('vins_relay')
        
        # Subscribe to Isaac Sim
        self.imu_sub = self.create_subscription(Imu, '/srb/env0/imu_robot', self.relay_imu, 10)
        self.img_sub = self.create_subscription(Image, '/srb/env0/cam_onboard/image_rgb', self.relay_img, 10)
        
        # Republish to what VINS expects
        self.imu_pub = self.create_publisher(Imu, '/vins/imu', 10)
        self.img_pub = self.create_publisher(Image, '/vins/cam0/image_raw', 10)
        
        self.get_logger().info('VINS Relay: Isaac Sim -> VINS topics')
        
    def relay_imu(self, msg):
        self.imu_pub.publish(msg)
        
    def relay_img(self, msg):
        self.img_pub.publish(msg)

def main():
    rclpy.init()
    node = VINSRelay()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
