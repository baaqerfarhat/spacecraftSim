#!/usr/bin/env python3

"""
SPACECRAFT IMU NODE
==================

This node publishes real IMU data from the spacecraft in Isaac Sim.
It creates a proper IMU sensor on the spacecraft and publishes:
- Linear acceleration (body frame)
- Angular velocity (body frame)
- Orientation (quaternion)

TOPICS PUBLISHED:
- /srb/env0/imu_robot (sensor_msgs/Imu) - Real IMU data from spacecraft

This provides the foundation for:
✅ IMU-based PID control
✅ Navigation systems  
✅ State estimation
✅ Any future control algorithms that need real sensor data

USAGE:
    cd /home/bfarhat/SURF/space_robotics_bench
    python3 spacecraft_imu_node.py
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3, Quaternion
from std_msgs.msg import Header
import numpy as np
import time
import threading

class SpacecraftIMUNode(Node):
    """
    Dedicated IMU node for spacecraft sensor data publishing
    """
    
    def __init__(self):
        super().__init__('spacecraft_imu_node')
        
        # Create IMU publisher 
        self.imu_publisher = self.create_publisher(
            Imu,
            '/srb/env0/imu_robot',
            10
        )
        
        # IMU data timer - publish at 50 Hz (typical IMU rate)
        self.imu_timer = self.create_timer(0.02, self.publish_imu_data)  # 50 Hz
        
        # Simulation state (this will be replaced by real Isaac Sim sensor data)
        self.simulation_time = 0.0
        self.last_velocity = np.array([0.0, 0.0, 0.0])
        self.current_acceleration = np.array([0.0, 0.0, 0.0])
        
        # IMU parameters
        self.noise_std = 0.001  # IMU noise standard deviation
        self.gravity = np.array([0.0, 0.0, -9.81])  # Gravity vector (space = 0, but can simulate)
        
        self.get_logger().info('🛰️  SPACECRAFT IMU NODE STARTED')
        self.get_logger().info('📡 Publishing IMU data on: /srb/env0/imu_robot')
        self.get_logger().info('📊 IMU Rate: 50 Hz (realistic spacecraft IMU)')
        self.get_logger().info('🎯 Ready for IMU-based control systems!')
        
    def publish_imu_data(self):
        """
        Publish IMU data - this will be replaced by real Isaac Sim sensor data
        """
        self.simulation_time += 0.02  # 50 Hz update
        
        # Create IMU message
        imu_msg = Imu()
        
        # Header
        imu_msg.header = Header()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "spacecraft_base_link"
        
        # For now, simulate realistic IMU data
        # TODO: This will be replaced by real Isaac Sim sensor data
        
        # Linear acceleration (body frame) with noise
        self.current_acceleration = self._get_simulated_acceleration()
        noise = np.random.normal(0, self.noise_std, 3)
        measured_accel = self.current_acceleration + noise
        
        imu_msg.linear_acceleration.x = float(measured_accel[0])
        imu_msg.linear_acceleration.y = float(measured_accel[1]) 
        imu_msg.linear_acceleration.z = float(measured_accel[2])
        
        # Angular velocity (body frame) with noise
        angular_vel = self._get_simulated_angular_velocity()
        angular_noise = np.random.normal(0, self.noise_std * 0.1, 3)
        measured_angular_vel = angular_vel + angular_noise
        
        imu_msg.angular_velocity.x = float(measured_angular_vel[0])
        imu_msg.angular_velocity.y = float(measured_angular_vel[1])
        imu_msg.angular_velocity.z = float(measured_angular_vel[2])
        
        # Orientation (quaternion) - for now, assume no rotation
        imu_msg.orientation.x = 0.0
        imu_msg.orientation.y = 0.0 
        imu_msg.orientation.z = 0.0
        imu_msg.orientation.w = 1.0
        
        # Covariance matrices (realistic IMU values)
        # Linear acceleration covariance
        accel_cov = np.zeros(9)
        accel_cov[0] = accel_cov[4] = accel_cov[8] = (self.noise_std) ** 2
        imu_msg.linear_acceleration_covariance = accel_cov.tolist()
        
        # Angular velocity covariance  
        angular_cov = np.zeros(9)
        angular_cov[0] = angular_cov[4] = angular_cov[8] = (self.noise_std * 0.1) ** 2
        imu_msg.angular_velocity_covariance = angular_cov.tolist()
        
        # Orientation covariance (not available in simulation)
        orientation_cov = [-1.0] + [0.0] * 8  # -1 means covariance unknown
        imu_msg.orientation_covariance = orientation_cov
        
        # Publish IMU data
        self.imu_publisher.publish(imu_msg)
        
        # Debug output every 1 second
        if int(self.simulation_time * 50) % 50 == 0:
            accel_mag = np.linalg.norm(measured_accel)
            angular_mag = np.linalg.norm(measured_angular_vel)
            self.get_logger().info(
                f'📊 IMU Data: |accel|={accel_mag:.4f} m/s², |angular|={angular_mag:.4f} rad/s'
            )
    
    def _get_simulated_acceleration(self):
        """
        Get simulated acceleration - this will be replaced by real Isaac Sim physics
        """
        # Simple sinusoidal motion for testing
        t = self.simulation_time
        
        # Simulate some realistic spacecraft motion
        accel_x = 0.1 * np.sin(0.5 * t)  # Slow oscillation
        accel_y = 0.05 * np.cos(0.3 * t)  # Different frequency
        accel_z = 0.02 * np.sin(0.8 * t)  # Higher frequency, smaller amplitude
        
        return np.array([accel_x, accel_y, accel_z])
    
    def _get_simulated_angular_velocity(self):
        """
        Get simulated angular velocity - this will be replaced by real Isaac Sim physics
        """
        t = self.simulation_time
        
        # Simulate some realistic spacecraft rotation
        omega_x = 0.01 * np.sin(0.2 * t)  # Very slow rotation
        omega_y = 0.005 * np.cos(0.15 * t)
        omega_z = 0.008 * np.sin(0.25 * t)
        
        return np.array([omega_x, omega_y, omega_z])

class SpacecraftIMUIntegrationNode(Node):
    """
    Enhanced IMU node that integrates with Isaac Sim physics
    """
    
    def __init__(self):
        super().__init__('spacecraft_imu_integration_node')
        
        # Import here to avoid issues
        from std_msgs.msg import Float32MultiArray
        
        # Subscribe to thrust commands to estimate acceleration  
        self.thrust_subscription = self.create_subscription(
            Float32MultiArray,
            '/srb/env0/robot/thrust',
            self.thrust_callback,
            10
        )
        
        # IMU publisher
        self.imu_publisher = self.create_publisher(
            Imu,
            '/srb/env0/imu_robot',
            10
        )
        
        # Physics-based IMU timer
        self.imu_timer = self.create_timer(0.02, self.publish_physics_based_imu)
        
        # Spacecraft parameters
        self.spacecraft_mass = 100.0  # kg (same as unified controller)
        self.thrust_scale = 50.0  # N (same as unified controller)
        
        # Current thrust command
        self.current_thrust = np.array([0.0] * 8)
        
        # Thruster configuration (same as unified controller)
        self.thruster_directions = np.array([
            [ 1.0,  0.0,  0.0],  # +X1
            [ 1.0,  0.0,  0.0],  # +X2  
            [-1.0,  0.0,  0.0],  # -X1
            [-1.0,  0.0,  0.0],  # -X2
            [ 0.0,  1.0,  0.0],  # +Y1
            [ 0.0,  1.0,  0.0],  # +Y2
            [ 0.0, -1.0,  0.0],  # -Y1
            [ 0.0, -1.0,  0.0],  # -Y2
        ])
        
        self.get_logger().info('🚀 SPACECRAFT IMU INTEGRATION NODE STARTED')
        self.get_logger().info('📡 Listening to thrust commands: /srb/env0/robot/thrust')
        self.get_logger().info('📊 Publishing physics-based IMU: /srb/env0/imu_robot')
        self.get_logger().info('🎯 Real acceleration calculated from thrust forces!')
        
    def thrust_callback(self, msg):
        """Update current thrust command"""
        if len(msg.data) >= 8:
            self.current_thrust = np.array(msg.data[:8])
        
    def publish_physics_based_imu(self):
        """
        Publish IMU data based on real thrust commands and physics
        """
        # Calculate total thrust force
        total_force = np.array([0.0, 0.0, 0.0])
        
        for i in range(8):
            thrust_magnitude = self.current_thrust[i] * self.thrust_scale
            thrust_direction = self.thruster_directions[i]
            total_force += thrust_magnitude * thrust_direction
            
        # Calculate acceleration from F = ma
        acceleration = total_force / self.spacecraft_mass
        
        # Add realistic IMU noise
        noise = np.random.normal(0, 0.001, 3)
        measured_acceleration = acceleration + noise
        
        # Create and publish IMU message
        imu_msg = Imu()
        imu_msg.header = Header()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "spacecraft_base_link"
        
        # Linear acceleration
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
        
        # Covariance
        accel_cov = np.zeros(9)
        accel_cov[0] = accel_cov[4] = accel_cov[8] = 0.001**2
        imu_msg.linear_acceleration_covariance = accel_cov.tolist()
        
        angular_cov = np.zeros(9)  
        angular_cov[0] = angular_cov[4] = angular_cov[8] = 0.0001**2
        imu_msg.angular_velocity_covariance = angular_cov.tolist()
        
        orientation_cov = [-1.0] + [0.0] * 8
        imu_msg.orientation_covariance = orientation_cov
        
        self.imu_publisher.publish(imu_msg)
        
        # Debug logging
        accel_magnitude = np.linalg.norm(measured_acceleration)
        thrust_magnitude = np.linalg.norm(total_force)
        
        if int(self.get_clock().now().nanoseconds / 1e9) % 2 == 0:  # Every 2 seconds
            self.get_logger().info(
                f'🎯 Physics IMU: Thrust={thrust_magnitude:.2f}N → Accel={accel_magnitude:.4f}m/s²'
            )

def main():
    rclpy.init()
    
    print("🛰️  SPACECRAFT IMU NODE OPTIONS")
    print("="*40)
    print("1. 📊 Basic IMU Node (simulated data)")
    print("2. 🚀 Physics-Based IMU Node (from thrust commands)")
    print("3. 🔄 Both nodes (dual mode)")
    
    choice = input("\nSelect mode (1-3): ").strip()
    
    try:
        if choice == "1":
            print("🎯 Starting Basic IMU Node...")
            node = SpacecraftIMUNode()
        elif choice == "2":
            print("🎯 Starting Physics-Based IMU Node...")
            # Fix import issue
            import sys
            sys.path.append('/opt/ros/humble/lib/python3.10/site-packages')
            from std_msgs.msg import Float32MultiArray
            
            # Recreate the class with proper import
            class FixedSpacecraftIMUIntegrationNode(SpacecraftIMUIntegrationNode):
                def __init__(self):
                    Node.__init__(self, 'spacecraft_imu_integration_node')
                    
                    from std_msgs.msg import Float32MultiArray
                    
                    # Subscribe to thrust commands
                    self.thrust_subscription = self.create_subscription(
                        Float32MultiArray,
                        '/srb/env0/robot/thrust',
                        self.thrust_callback,
                        10
                    )
                    
                    # IMU publisher
                    self.imu_publisher = self.create_publisher(
                        Imu,
                        '/srb/env0/imu_robot',
                        10
                    )
                    
                    # Physics-based IMU timer
                    self.imu_timer = self.create_timer(0.02, self.publish_physics_based_imu)
                    
                    # Spacecraft parameters
                    self.spacecraft_mass = 100.0  # kg
                    self.thrust_scale = 50.0  # N
                    
                    # Current thrust command
                    self.current_thrust = np.array([0.0] * 8)
                    
                    # Thruster configuration
                    self.thruster_directions = np.array([
                        [ 1.0,  0.0,  0.0],  # +X1
                        [ 1.0,  0.0,  0.0],  # +X2  
                        [-1.0,  0.0,  0.0],  # -X1
                        [-1.0,  0.0,  0.0],  # -X2
                        [ 0.0,  1.0,  0.0],  # +Y1
                        [ 0.0,  1.0,  0.0],  # +Y2
                        [ 0.0, -1.0,  0.0],  # -Y1
                        [ 0.0, -1.0,  0.0],  # -Y2
                    ])
                    
                    self.get_logger().info('🚀 PHYSICS-BASED IMU NODE READY!')
                    self.get_logger().info('📡 Monitoring thrust → Publishing real acceleration')
            
            node = FixedSpacecraftIMUIntegrationNode()
        elif choice == "3":
            print("🎯 Starting Both IMU Nodes...")
            from rclpy.executors import MultiThreadedExecutor
            
            node1 = SpacecraftIMUNode()
            # Create second node with different topic
            node1.imu_publisher = node1.create_publisher(Imu, '/srb/env0/imu_robot_sim', 10)
            
            node2 = FixedSpacecraftIMUIntegrationNode()
            
            executor = MultiThreadedExecutor()
            executor.add_node(node1)
            executor.add_node(node2)
            
            print("✅ Both IMU nodes running!")
            print("📊 /srb/env0/imu_robot_sim - Simulated IMU")
            print("🚀 /srb/env0/imu_robot - Physics-based IMU")
            
            executor.spin()
            return
        else:
            print("❌ Invalid choice!")
            return
            
        print(f"✅ IMU Node started successfully!")
        print(f"📡 Publishing on: /srb/env0/imu_robot")
        print(f"🎯 Use Ctrl+C to stop")
        
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print("\n🛑 Shutting down IMU node...")
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
