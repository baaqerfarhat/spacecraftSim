import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, Imu, CameraInfo
from std_msgs.msg import Header
import message_filters
from cv_bridge import CvBridge
import cv2


class VINSBridge(Node):
    def __init__(self):
        super().__init__('vins_bridge')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # Inputs from SRB (env0 by default)
        cam_topic = self.declare_parameter('cam_topic', '/srb/env0/cam_onboard/image_rgb').get_parameter_value().string_value
        cam_info_topic = self.declare_parameter('cam_info_topic', '/srb/env0/cam_onboard/camera_info').get_parameter_value().string_value
        imu_topic = self.declare_parameter('imu_topic', '/srb/env0/imu_robot').get_parameter_value().string_value
        convert_to_mono = self.declare_parameter('convert_to_mono', True).get_parameter_value().bool_value

        # Outputs for VINS-Fusion
        vins_ns = self.declare_parameter('vins_ns', '/vins').get_parameter_value().string_value
        img_out_topic = f"{vins_ns}/cam0/image_raw"
        cam_info_out_topic = f"{vins_ns}/cam0/camera_info"
        imu_out_topic = f"{vins_ns}/imu"

        # Publishers
        self.pub_img = self.create_publisher(Image, img_out_topic, qos)
        self.pub_info = self.create_publisher(CameraInfo, cam_info_out_topic, qos)
        self.pub_imu = self.create_publisher(Imu, imu_out_topic, qos)
        self.convert_to_mono = convert_to_mono
        self.bridge = CvBridge()

        # Subscriptions with time sync for image + camera_info
        img_sub = message_filters.Subscriber(self, Image, cam_topic, qos_profile=qos)
        info_sub = message_filters.Subscriber(self, CameraInfo, cam_info_topic, qos_profile=qos)
        self.ts = message_filters.ApproximateTimeSynchronizer([img_sub, info_sub], queue_size=10, slop=0.05)
        self.ts.registerCallback(self.cb_image)

        self.sub_imu = self.create_subscription(Imu, imu_topic, self.cb_imu, qos)

        self.get_logger().info(f"VINS Bridge: IMU {imu_topic} -> {imu_out_topic}")
        self.get_logger().info(f"VINS Bridge: IMG {cam_topic} (+info) -> {img_out_topic}")

    def cb_image(self, img: Image, info: CameraInfo):
        # OpenVINS expects consistent frame_ids and timestamps
        # Pass-through, ensure header frame IDs are set to ov frames
        img_hdr = Header()
        img_hdr.stamp = img.header.stamp
        img_hdr.frame_id = 'cam0'

        info_hdr = Header()
        info_hdr.stamp = info.header.stamp
        info_hdr.frame_id = 'cam0'

        # Optionally convert to mono8 using OpenCV, as many VIO stacks expect grayscale
        if self.convert_to_mono and img.encoding != 'mono8':
            try:
                cv_img = self.bridge.imgmsg_to_cv2(img, desired_encoding=('bgr8' if img.encoding in ('rgb8', 'bgr8') else img.encoding))
                if len(cv_img.shape) == 3:
                    gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
                else:
                    gray = cv_img
                img = self.bridge.cv2_to_imgmsg(gray, encoding='mono8')
            except Exception as e:
                self.get_logger().warn(f"Failed image conversion, forwarding original: {e}")
        img.header = img_hdr
        info.header = info_hdr

        self.pub_img.publish(img)
        self.pub_info.publish(info)

    def cb_imu(self, imu: Imu):
        imu.header.frame_id = 'imu0'
        self.pub_imu.publish(imu)


def main():
    rclpy.init()
    node = VINSBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


