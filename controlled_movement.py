#!/usr/bin/env python3
"""
Controlled Spacecraft Movement Script
Moves the spacecraft to the right for a specific distance and then stops.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped
import time
import math


class SpacecraftController(Node):
    def __init__(self):
        super().__init__('spacecraft_controller')
        
        # Create publisher for thruster commands
        self.thruster_pub = self.create_publisher(
            Float32MultiArray, 
            '/srb/env0/robot/thrust', 
            10
        )
        
        # Create subscriber for robot pose
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/srb/env0/robot/pose',  # You may need to check the actual topic name
            self.pose_callback,
            10
        )
        
        # Movement parameters
        self.target_distance = 5.0  # meters to move right
        self.thrust_magnitude = 0.3  # 30% thrust power
        self.position_tolerance = 0.1  # meters
        self.velocity_tolerance = 0.05  # m/s
        
        # State variables
        self.initial_position = None
        self.current_position = None
        self.current_velocity = None
        self.last_position = None
        self.last_time = None
        self.movement_started = False
        self.movement_complete = False
        
        # Thruster configuration based on your setup
        # Thrusters 0,1 point +X (right), thrusters 2,3 point -X (left)
        self.RIGHT_THRUSTERS = [0, 1]  # +X direction
        self.LEFT_THRUSTERS = [2, 3]   # -X direction
        
        self.get_logger().info("Spacecraft Controller initialized")
        self.get_logger().info(f"Target distance: {self.target_distance}m to the right")
        
        # Wait a bit for pose subscription to initialize
        time.sleep(1.0)
        
    def pose_callback(self, msg):
        """Handle incoming pose messages to track position"""
        current_time = time.time()
        
        # Extract position
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        
        self.current_position = [x, y, z]
        
        # Calculate velocity if we have previous data
        if self.last_position is not None and self.last_time is not None:
            dt = current_time - self.last_time
            if dt > 0:
                vx = (x - self.last_position[0]) / dt
                vy = (y - self.last_position[1]) / dt
                vz = (z - self.last_position[2]) / dt
                self.current_velocity = [vx, vy, vz]
        
        # Store for next iteration
        self.last_position = self.current_position.copy()
        self.last_time = current_time
        
        # Set initial position reference
        if self.initial_position is None:
            self.initial_position = self.current_position.copy()
            self.get_logger().info(f"Initial position set: {self.initial_position}")
            
    def send_thruster_command(self, thrust_values):
        """Send thruster command with 8 values (one per thruster)"""
        msg = Float32MultiArray()
        msg.data = thrust_values
        self.thruster_pub.publish(msg)
        
    def stop_all_thrusters(self):
        """Stop all thrusters"""
        self.send_thruster_command([0.0] * 8)
        self.get_logger().info("All thrusters stopped")
        
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
        
    def get_distance_traveled(self):
        """Calculate distance traveled in X direction from start"""
        if self.initial_position is None or self.current_position is None:
            return 0.0
        return self.current_position[0] - self.initial_position[0]
        
    def get_x_velocity(self):
        """Get current X velocity"""
        if self.current_velocity is None:
            return 0.0
        return self.current_velocity[0]
        
    def run_movement(self):
        """Main control loop"""
        rate = self.create_rate(20)  # 20 Hz control loop
        
        # Wait for initial position
        while self.initial_position is None:
            self.get_logger().info("Waiting for initial position...")
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)
            
        self.get_logger().info("Starting controlled movement...")
        
        phase = "ACCELERATE"
        
        while rclpy.ok() and not self.movement_complete:
            rclpy.spin_once(self, timeout_sec=0.01)
            
            if self.current_position is None:
                continue
                
            distance_traveled = self.get_distance_traveled()
            x_velocity = self.get_x_velocity()
            
            # Log current state
            if hasattr(self, '_log_counter'):
                self._log_counter += 1
            else:
                self._log_counter = 0
                
            if self._log_counter % 20 == 0:  # Log every second
                self.get_logger().info(
                    f"Phase: {phase} | Distance: {distance_traveled:.3f}m | "
                    f"Velocity: {x_velocity:.3f}m/s | Target: {self.target_distance}m"
                )
            
            # Control logic
            distance_remaining = self.target_distance - distance_traveled
            
            if phase == "ACCELERATE":
                if distance_remaining <= 0.5:  # Start braking when close
                    phase = "BRAKE"
                    self.get_logger().info("Switching to BRAKE phase")
                else:
                    self.move_right(self.thrust_magnitude)
                    
            elif phase == "BRAKE":
                if (abs(distance_remaining) < self.position_tolerance and 
                    abs(x_velocity) < self.velocity_tolerance):
                    phase = "COMPLETE"
                    self.get_logger().info("Movement complete!")
                else:
                    # Brake with proportional control
                    brake_magnitude = min(self.thrust_magnitude, abs(x_velocity) * 2.0)
                    self.brake_right(brake_magnitude)
                    
            elif phase == "COMPLETE":
                self.stop_all_thrusters()
                self.movement_complete = True
                break
                
            rate.sleep()
            
        # Final stop
        self.stop_all_thrusters()
        final_distance = self.get_distance_traveled()
        self.get_logger().info(f"Movement completed. Final distance: {final_distance:.3f}m")


def main():
    rclpy.init()
    
    try:
        controller = SpacecraftController()
        controller.run_movement()
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
