#!/usr/bin/env python3
"""
Manual Spacecraft Commands
Simple script to send individual thruster commands for testing.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import time


class ManualCommander(Node):
    def __init__(self):
        super().__init__('manual_commander')
        
        # Create publisher for thruster commands
        self.thruster_pub = self.create_publisher(
            Float32MultiArray, 
            '/srb/env0/robot/thrust', 
            10
        )
        
        # Thruster configuration
        self.RIGHT_THRUSTERS = [0, 1]  # +X direction (right)
        self.LEFT_THRUSTERS = [2, 3]   # -X direction (left) 
        self.FORWARD_THRUSTERS = [4, 5] # +Y direction (forward)
        self.BACK_THRUSTERS = [6, 7]    # -Y direction (backward)
        
        self.get_logger().info("Manual Commander initialized")
        self.get_logger().info("Thruster layout:")
        self.get_logger().info("  Right (0,1): +X direction")
        self.get_logger().info("  Left (2,3): -X direction") 
        self.get_logger().info("  Forward (4,5): +Y direction")
        self.get_logger().info("  Back (6,7): -Y direction")
        
    def send_command(self, thrust_values, description=""):
        """Send thruster command"""
        msg = Float32MultiArray()
        msg.data = thrust_values
        self.thruster_pub.publish(msg)
        self.get_logger().info(f"{description}: {thrust_values}")
        
    def stop(self):
        """Stop all thrusters"""
        self.send_command([0.0] * 8, "STOP")
        
    def move_right(self, power=0.3):
        """Move right"""
        cmd = [0.0] * 8
        for i in self.RIGHT_THRUSTERS:
            cmd[i] = power
        self.send_command(cmd, f"MOVE RIGHT ({power})")
        
    def move_left(self, power=0.3):
        """Move left (brake right motion)"""
        cmd = [0.0] * 8
        for i in self.LEFT_THRUSTERS:
            cmd[i] = power
        self.send_command(cmd, f"MOVE LEFT ({power})")
        
    def move_forward(self, power=0.3):
        """Move forward"""
        cmd = [0.0] * 8
        for i in self.FORWARD_THRUSTERS:
            cmd[i] = power
        self.send_command(cmd, f"MOVE FORWARD ({power})")
        
    def move_back(self, power=0.3):
        """Move backward"""
        cmd = [0.0] * 8
        for i in self.BACK_THRUSTERS:
            cmd[i] = power
        self.send_command(cmd, f"MOVE BACK ({power})")


def main():
    rclpy.init()
    
    try:
        commander = ManualCommander()
        time.sleep(1.0)  # Wait for publisher
        
        print("\n=== CONTROLLED RIGHT MOVEMENT TEST ===")
        
        # Move right for 3 seconds
        print("1. Moving RIGHT for 3 seconds...")
        commander.move_right(0.3)
        time.sleep(3.0)
        
        # Brake for 2 seconds
        print("2. Braking (LEFT thrust) for 2 seconds...")
        commander.move_left(0.3)
        time.sleep(2.0)
        
        # Stop
        print("3. Stopping...")
        commander.stop()
        time.sleep(1.0)
        
        print("=== MOVEMENT COMPLETE ===")
        
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'commander' in locals():
            commander.stop()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
