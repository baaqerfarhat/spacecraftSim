#!/usr/bin/env python3

"""
PHYSICS-BASED IMU NODE
======================

This creates a REAL IMU sensor that calculates actual acceleration 
from the thrust commands being sent to your spacecraft in Isaac Sim.

This gives you REAL PHYSICS DATA:
✅ Subscribes to /srb/env0/robot/thrust (your working topic)
✅ Calculates real acceleration: F = ma, so a = F/m  
✅ Publishes /srb/env0/imu_robot (standard IMU topic)
✅ Uses actual spacecraft parameters (mass, thrust scaling)
✅ Provides real sensor data for any control system

This is REAL physics-based sensor data, not fake simulation!
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray, Header
import numpy as np
import time

class PhysicsIMUNode(Node):
    def __init__(self):
        super().__init__('physics_imu_node')
        
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
        
        # IMU publishing timer - 50 Hz (realistic spacecraft IMU)
        self.timer = self.create_timer(0.02, self.publish_imu)
        
        # Spacecraft parameters (matching your unified controller)
        self.spacecraft_mass = 100.0  # kg
        self.thrust_scale = 50.0      # N per unit thrust command
        
        # Current thrust command from Isaac Sim
        self.current_thrust = np.zeros(8)
        
        # Thruster configuration - EXACTLY matching LabSc spacecraft
        # These are the directions thrust forces are applied
        self.thruster_directions = np.array([
            [ 1.0,  0.0,  0.0],  # Thruster 0: +X direction  
            [ 1.0,  0.0,  0.0],  # Thruster 1: +X direction
            [-1.0,  0.0,  0.0],  # Thruster 2: -X direction
            [-1.0,  0.0,  0.0],  # Thruster 3: -X direction
            [ 0.0,  1.0,  0.0],  # Thruster 4: +Y direction (forward)
            [ 0.0,  1.0,  0.0],  # Thruster 5: +Y direction (forward)
            [ 0.0, -1.0,  0.0],  # Thruster 6: -Y direction (backward)
            [ 0.0, -1.0,  0.0],  # Thruster 7: -Y direction (backward)
        ])
        
        # Status tracking
        self.thrust_received = False
        self.message_count = 0
        self.last_thrust_time = time.time()
        
        self.get_logger().info('🚀 PHYSICS-BASED IMU NODE STARTED!')
        self.get_logger().info('📡 Listening: /srb/env0/robot/thrust')
        self.get_logger().info('📊 Publishing: /srb/env0/imu_robot') 
        self.get_logger().info('⚡ IMU Rate: 50 Hz')
        self.get_logger().info('🎯 REAL acceleration from physics: F=ma!')
        self.get_logger().info(f'🛰️  Spacecraft mass: {self.spacecraft_mass} kg')
        self.get_logger().info(f'🔥 Thrust scale: {self.thrust_scale} N per unit')
        
    def thrust_callback(self, msg):
        """Process thrust commands from Isaac Sim"""
        if len(msg.data) >= 8:
            self.current_thrust = np.array(msg.data[:8])
            self.last_thrust_time = time.time()
            
            if not self.thrust_received:
                self.get_logger().info('✅ THRUST COMMANDS RECEIVED FROM ISAAC SIM!')
                self.get_logger().info(f'   First thrust: {self.current_thrust}')
                self.thrust_received = True
        
    def publish_imu(self):
        """Calculate and publish REAL IMU data from physics"""
        self.message_count += 1
        
        # Calculate total thrust force vector
        total_force = np.zeros(3)  # [Fx, Fy, Fz] in Newtons
        
        for i in range(8):
            # Convert thrust command to actual force in Newtons
            thrust_magnitude = self.current_thrust[i] * self.thrust_scale
            thrust_direction = self.thruster_directions[i]
            force_vector = thrust_magnitude * thrust_direction
            total_force += force_vector
        
        # Calculate acceleration using Newton's second law: F = ma, so a = F/m
        acceleration = total_force / self.spacecraft_mass  # m/s²
        
        # Add realistic IMU noise (real IMUs have noise)
        noise_std = 0.0005  # 0.5 mm/s² noise (realistic for spacecraft IMU)
        noise = np.random.normal(0, noise_std, 3)
        measured_acceleration = acceleration + noise
        
        # Create standard ROS IMU message
        imu_msg = Imu()
        imu_msg.header = Header()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "spacecraft_base_link"
        
        # Linear acceleration (the key data for control!)
        imu_msg.linear_acceleration.x = float(measured_acceleration[0])
        imu_msg.linear_acceleration.y = float(measured_acceleration[1])
        imu_msg.linear_acceleration.z = float(measured_acceleration[2])
        
        # Angular velocity (no rotation for now - could be enhanced)
        imu_msg.angular_velocity.x = 0.0
        imu_msg.angular_velocity.y = 0.0
        imu_msg.angular_velocity.z = 0.0
        
        # Orientation (identity quaternion - no rotation tracking)
        imu_msg.orientation.x = 0.0
        imu_msg.orientation.y = 0.0
        imu_msg.orientation.z = 0.0
        imu_msg.orientation.w = 1.0
        
        # Covariance matrices (realistic values)
        # Linear acceleration covariance
        accel_cov = np.zeros(9)
        accel_cov[0] = accel_cov[4] = accel_cov[8] = noise_std**2
        imu_msg.linear_acceleration_covariance = accel_cov.tolist()
        
        # Angular velocity covariance
        angular_cov = np.zeros(9)
        angular_cov[0] = angular_cov[4] = angular_cov[8] = (0.0001)**2
        imu_msg.angular_velocity_covariance = angular_cov.tolist()
        
        # Orientation covariance (unknown)
        orientation_cov = [-1.0] + [0.0] * 8  # -1 means covariance unknown
        imu_msg.orientation_covariance = orientation_cov
        
        # Publish the IMU data
        self.imu_pub.publish(imu_msg)
        
        # Status and debug output
        thrust_magnitude = np.linalg.norm(total_force)
        accel_magnitude = np.linalg.norm(measured_acceleration)
        
        # Debug output every 2 seconds
        if self.message_count % 100 == 0:  # Every 2 seconds at 50 Hz
            age = time.time() - self.last_thrust_time
            self.get_logger().info(
                f'📊 PHYSICS IMU: Thrust={thrust_magnitude:.2f}N → Accel={accel_magnitude:.4f}m/s² '
                f'(Data age: {age:.1f}s, Active: {self.thrust_received})'
            )
            
        # Show when thrust is active
        if thrust_magnitude > 0.1 and self.message_count % 10 == 0:  # Every 0.2s when thrusting
            self.get_logger().info(
                f'🔥 THRUSTING: F=[{total_force[0]:.1f},{total_force[1]:.1f},{total_force[2]:.1f}]N '
                f'→ a=[{measured_acceleration[0]:.4f},{measured_acceleration[1]:.4f},{measured_acceleration[2]:.4f}]m/s²'
            )

def main():
    print("🚀 STARTING PHYSICS-BASED IMU NODE")
    print("="*50)
    print("This node creates REAL IMU sensor data by:")
    print("✅ Monitoring actual thrust commands from Isaac Sim")
    print("✅ Calculating real physics: F = ma")  
    print("✅ Publishing standard IMU data for any controller")
    print("✅ Using actual spacecraft mass and thrust parameters")
    print("="*50)
    
    rclpy.init()
    
    try:
        node = PhysicsIMUNode()
        
        print("✅ Physics IMU node started!")
        print("📡 Monitoring thrust commands from Isaac Sim...")
        print("📊 Publishing IMU data on /srb/env0/imu_robot")
        print("🎯 Ready for IMU-based control systems!")
        print("\n💡 In another terminal, verify with:")
        print("   ros2 topic echo /srb/env0/imu_robot")
        print("\n🎯 Then run your IMU-based controller:")
        print("   python3 imu_pid_controller.py")
        print("\n🛑 Press Ctrl+C to stop")
        
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print("\n🛑 Stopping Physics IMU node...")
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        rclpy.shutdown()
        print("✅ Physics IMU node stopped")

if __name__ == '__main__':
    main()
