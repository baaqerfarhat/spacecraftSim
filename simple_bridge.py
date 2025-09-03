import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu, CameraInfo

class SimpleBridge(Node):
    def __init__(self):
        super().__init__('simple_bridge')
        
        # Subscribe to Isaac Sim topics
        self.imu_sub = self.create_subscription(Imu, '/srb/env0/imu_robot', self.imu_callback, 10)
        self.img_sub = self.create_subscription(Image, '/srb/env0/cam_onboard/image_rgb', self.img_callback, 10)
        self.info_sub = self.create_subscription(CameraInfo, '/srb/env0/cam_onboard/camera_info', self.info_callback, 10)
        
        # Republish for VINS
        self.imu_pub = self.create_publisher(Imu, '/vins/imu', 10)
        self.img_pub = self.create_publisher(Image, '/vins/cam0/image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/vins/cam0/camera_info', 10)
        
        self.get_logger().info('Simple Bridge: Relaying Isaac Sim -> VINS topics')
        
    def imu_callback(self, msg):
        msg.header.frame_id = 'imu0'
        self.imu_pub.publish(msg)
        
    def img_callback(self, msg):
        msg.header.frame_id = 'cam0'
        self.img_pub.publish(msg)
        
    def info_callback(self, msg):
        msg.header.frame_id = 'cam0'
        self.info_pub.publish(msg)

def main():
    rclpy.init()
    node = SimpleBridge()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
