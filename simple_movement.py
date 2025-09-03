#!/usr/bin/env python3
"""
Simple Time-Based Spacecraft Movement Script
Moves the spacecraft to the right for a specific time and then stops.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import time


class SimpleSpacecraftController(Node):
    def __init__(self):
        super().__init__('simple_spacecraft_controller')
        
        # Create publisher for thruster commands
        self.thruster_pub = self.create_publisher(
            Float32MultiArray, 
            '/srb/env0/robot/thrust', 
            10
        )
        
        # Movement parameters
        self.thrust_magnitude = 0.3  # 30% thrust power
        self.thrust_time = 3.0       # seconds to thrust
        self.brake_time = 2.0        # seconds to brake
        self.settle_time = 1.0       # seconds to ensure stop
        
        # Thruster configuration based on your setup
        # Thrusters 0,1 point +X (right), thrusters 2,3 point -X (left)
        self.RIGHT_THRUSTERS = [0, 1]  # +X direction  
        self.LEFT_THRUSTERS = [2, 3]   # -X direction
        
        self.get_logger().info("Simple Spacecraft Controller initialized")
        self.get_logger().info(f"Will thrust right for {self.thrust_time}s, then brake for {self.brake_time}s")
        
    def send_thruster_command(self, thrust_values):
        """Send thruster command with 8 values (one per thruster)"""
        msg = Float32MultiArray()
        msg.data = thrust_values
        self.thruster_pub.publish(msg)
        self.get_logger().info(f"Sent command: {thrust_values}")
        
    def stop_all_thrusters(self):
        """Stop all thrusters"""
        self.send_thruster_command([0.0] * 8)
        
    def move_right(self, thrust_magnitude):
        """Activate right-pointing thrusters"""
        thrust_cmd = [0.0] * 8
        for thruster_idx in self.RIGHT_THRUSTERS:
            thrust_cmd[thruster_idx] = thrust_magnitude
        self.send_thruster_command(thrust_cmd)
        
    def brake_right(self, thrust_magnitude):
        """Activate left-pointing thrusters to brake rightward motion"""
        thrust_cmd = [0.0] * 8
        for thruster_idx in self.LEFT_THRUSTERS:
            thrust_cmd[thruster_idx] = thrust_magnitude
        self.send_thruster_command(thrust_cmd)
        
    def run_movement_sequence(self):
        """Execute the movement sequence"""
        try:
            # Phase 1: Accelerate right
            self.get_logger().info("Phase 1: Accelerating right...")
            self.move_right(self.thrust_magnitude)
            time.sleep(self.thrust_time)
            
            # Phase 2: Brake
            self.get_logger().info("Phase 2: Braking...")
            self.brake_right(self.thrust_magnitude)
            time.sleep(self.brake_time)
            
            # Phase 3: Stop and settle
            self.get_logger().info("Phase 3: Stopping...")
            self.stop_all_thrusters()
            time.sleep(self.settle_time)
            
            self.get_logger().info("Movement sequence complete!")
            
        except Exception as e:
            self.get_logger().error(f"Error during movement: {e}")
            self.stop_all_thrusters()


def main():
    rclpy.init()
    
    try:
        controller = SimpleSpacecraftController()
        
        # Wait a moment for publisher to initialize
        time.sleep(1.0)
        
        # Run the movement sequence
        controller.run_movement_sequence()
        
        # Keep node alive briefly
        time.sleep(1.0)
        
    except KeyboardInterrupt:
        print("\nMovement interrupted by user")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'controller' in locals():
            controller.stop_all_thrusters()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
