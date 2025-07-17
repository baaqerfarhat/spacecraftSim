#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import torch

class VisionPublisher(Node):
    def __init__(self):
        super().__init__('vision_publisher')
        self.publisher_ = self.create_publisher(Image, 'vision_topic', 10)
        self.bridge = CvBridge()

        # Example dummy image to test publishing
        dummy_img = torch.zeros((480, 640, 3), dtype=torch.uint8).numpy()
        msg = self.bridge.cv2_to_imgmsg(dummy_img, encoding='bgr8')
        self.publisher_.publish(msg)
        self.get_logger().info('Published dummy image.')

def main(args=None):
    rclpy.init(args=args)
    node = VisionPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
