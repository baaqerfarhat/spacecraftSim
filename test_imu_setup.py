#!/usr/bin/env python3

"""
TEST IMU SETUP
==============

Simple script to test IMU data reception without the ROS topic list issues.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import time

class IMUTester(Node):
    def __init__(self):
        super().__init__('imu_tester')
        
        self.subscription = self.create_subscription(
            Imu,
            '/srb/env0/imu_robot',
            self.imu_callback,
            10
        )
        
        self.data_received = False
        self.message_count = 0
        
    def imu_callback(self, msg):
        self.message_count += 1
        
        if not self.data_received:
            print("✅ IMU DATA RECEIVED!")
            self.data_received = True
            
        if self.message_count % 25 == 0:  # Every 0.5 seconds at 50 Hz
            accel = msg.linear_acceleration
            print(f"📊 IMU #{self.message_count}: Accel=[{accel.x:.4f}, {accel.y:.4f}, {accel.z:.4f}] m/s²")

def main():
    print("🔍 TESTING IMU DATA RECEPTION")
    print("Listening on /srb/env0/imu_robot for 10 seconds...")
    
    rclpy.init()
    
    try:
        tester = IMUTester()
        
        # Listen for 10 seconds
        start_time = time.time()
        while time.time() - start_time < 10.0:
            rclpy.spin_once(tester, timeout_sec=0.1)
            
        if tester.data_received:
            print(f"✅ SUCCESS! Received {tester.message_count} IMU messages")
            print("🎯 IMU system working! You can now use imu_pid_controller.py")
        else:
            print("❌ No IMU data received")
            print("💡 Make sure quick_start_imu.py is running")
            
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
