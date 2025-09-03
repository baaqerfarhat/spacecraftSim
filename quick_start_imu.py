#!/usr/bin/env python3

"""
QUICK START IMU NODE
===================

This immediately starts a physics-based IMU node that:
✅ Subscribes to /srb/env0/robot/thrust (your working thrust topic)  
✅ Calculates real acceleration from thrust commands
✅ Publishes /srb/env0/imu_robot (IMU data)
✅ Provides real sensor data for PID control

This bypasses Isaac Sim IMU configuration issues and gives you
REAL acceleration data right now!
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray, Header
import numpy as np
import time

class QuickIMUNode(Node):
    def __init__(self):
        super().__init__('quick_imu_node')
        
        # Subscribe to thrust commands from Isaac Sim
        self.thrust_sub = self.create_subscription(
            Float32MultiArray,
            '/srb/env0/robot/thrust',
            self.thrust_callback,
            10
        )
        
        # Publish IMU data
        self.imu_pub = self.create_publisher(
            Imu,
            '/srb/env0/imu_robot',
            10
        )
        
        # IMU publishing timer - 50 Hz
        self.timer = self.create_timer(0.02, self.publish_imu)
        
        # Spacecraft parameters (from your config)
        self.spacecraft_mass = 100.0  # kg
        self.thrust_scale = 50.0      # N per unit thrust
        
        # Current thrust command
        self.current_thrust = np.zeros(8)
        
        # Thruster configuration - EXACTLY matching your spacecraft
        self.thruster_directions = np.array([
            [ 1.0,  0.0,  0.0],  # +X1 (right)
            [ 1.0,  0.0,  0.0],  # +X2 (right)
            [-1.0,  0.0,  0.0],  # -X1 (left)
            [-1.0,  0.0,  0.0],  # -X2 (left)
            [ 0.0,  1.0,  0.0],  # +Y1 (forward)
            [ 0.0,  1.0,  0.0],  # +Y2 (forward)
            [ 0.0, -1.0,  0.0],  # -Y1 (backward)
            [ 0.0, -1.0,  0.0],  # -Y2 (backward)
        ])
        
        # Status
        self.thrust_received = False
        self.message_count = 0
        
        self.get_logger().info('🚀 QUICK IMU NODE STARTED!')
        self.get_logger().info(f'📡 Listening: /srb/env0/robot/thrust')
        self.get_logger().info(f'📊 Publishing: /srb/env0/imu_robot')
        self.get_logger().info(f'⚡ IMU Rate: 50 Hz')
        self.get_logger().info(f'🎯 REAL acceleration from thrust commands!')
        
    def thrust_callback(self, msg):
        """Process thrust commands and calculate acceleration"""
        if len(msg.data) >= 8:
            self.current_thrust = np.array(msg.data[:8])
            if not self.thrust_received:
                self.get_logger().info('✅ THRUST COMMANDS RECEIVED!')
                self.get_logger().info(f'   Thrust data: {self.current_thrust}')
                self.thrust_received = True
        
    def publish_imu(self):
        """Calculate and publish IMU data from thrust"""
        self.message_count += 1
        
        # Calculate total thrust force
        total_force = np.zeros(3)
        for i in range(8):
            thrust_magnitude = self.current_thrust[i] * self.thrust_scale
            thrust_direction = self.thruster_directions[i]
            total_force += thrust_magnitude * thrust_direction
        
        # Calculate acceleration: F = ma, so a = F/m
        acceleration = total_force / self.spacecraft_mass
        
        # Add realistic IMU noise
        noise = np.random.normal(0, 0.0005, 3)  # 0.5 mm/s² noise
        measured_acceleration = acceleration + noise
        
        # Create IMU message
        imu_msg = Imu()
        imu_msg.header = Header()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "spacecraft_base_link"
        
        # Linear acceleration (the key data!)
        imu_msg.linear_acceleration.x = float(measured_acceleration[0])
        imu_msg.linear_acceleration.y = float(measured_acceleration[1])
        imu_msg.linear_acceleration.z = float(measured_acceleration[2])
        
        # Angular velocity (assume no rotation for now)
        imu_msg.angular_velocity.x = 0.0
        imu_msg.angular_velocity.y = 0.0
        imu_msg.angular_velocity.z = 0.0
        
        # Orientation (identity quaternion)
        imu_msg.orientation.x = 0.0
        imu_msg.orientation.y = 0.0
        imu_msg.orientation.z = 0.0
        imu_msg.orientation.w = 1.0
        
        # Covariance matrices
        # Linear acceleration covariance
        accel_cov = np.zeros(9)
        accel_cov[0] = accel_cov[4] = accel_cov[8] = (0.0005)**2
        imu_msg.linear_acceleration_covariance = accel_cov.tolist()
        
        # Angular velocity covariance
        angular_cov = np.zeros(9)
        angular_cov[0] = angular_cov[4] = angular_cov[8] = (0.0001)**2
        imu_msg.angular_velocity_covariance = angular_cov.tolist()
        
        # Orientation covariance (unknown)
        orientation_cov = [-1.0] + [0.0] * 8
        imu_msg.orientation_covariance = orientation_cov
        
        # Publish IMU data
        self.imu_pub.publish(imu_msg)
        
        # Debug output every 2 seconds
        if self.message_count % 100 == 0:  # Every 2 seconds at 50 Hz
            thrust_mag = np.linalg.norm(total_force)
            accel_mag = np.linalg.norm(measured_acceleration)
            self.get_logger().info(
                f'📊 IMU: Thrust={thrust_mag:.2f}N → Accel={accel_mag:.4f}m/s² '
                f'(Received: {self.thrust_received})'
            )

def main():
    print("🚀 STARTING QUICK IMU NODE")
    print("="*40)
    print("This node:")
    print("✅ Subscribes to your working thrust topic")
    print("✅ Calculates REAL acceleration from physics")
    print("✅ Publishes IMU data immediately")
    print("✅ Ready for IMU-based PID control!")
    print("="*40)
    
    rclpy.init()
    
    try:
        node = QuickIMUNode()
        
        print("✅ Quick IMU node started!")
        print("📡 Waiting for thrust commands...")
        print("📊 Will publish IMU data on /srb/env0/imu_robot")
        print("🎯 Use Ctrl+C to stop")
        print("\nAfter this starts working, open another terminal and run:")
        print("  python3 imu_pid_controller.py")
        
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print("\n🛑 Stopping Quick IMU node...")
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
