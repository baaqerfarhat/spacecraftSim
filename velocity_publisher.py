#!/usr/bin/env python3
"""
Velocity Publisher for SRB Spacecraft

This node extracts velocity information from SRB and publishes it for the PID controller.
It can work in two modes:
1. Real mode: Extract from actual SRB topics (when available)  
2. Mock mode: Simulate based on thrust commands for testing
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import JointState
import numpy as np
import time


class VelocityPublisher(Node):
    def __init__(self):
        super().__init__('velocity_publisher')
        
        # Publishers
        self.velocity_pub = self.create_publisher(
            Vector3,
            '/spacecraft/velocity',
            10
        )
        
        # Subscribers
        self.thrust_sub = self.create_subscription(
            Float32MultiArray,
            '/srb/env0/robot/thrust',
            self.thrust_callback,
            10
        )
        
        # Try to subscribe to actual robot state (if available)
        self.joint_states_sub = self.create_subscription(
            JointState,
            '/srb/env0/robot/joint_states',
            self.joint_states_callback,
            10
        )
        
        # Physics simulation parameters for mock mode
        self.spacecraft_mass = 100.0  # kg
        self.drag_coefficient = 0.98  # Simulated drag for stability
        self.thrust_scale = 1000.0    # Convert thrust to Newtons
        
        # Current state
        self.velocity = np.array([0.0, 0.0, 0.0])  # [x, y, z] m/s
        self.last_thrust_command = np.array([0.0] * 8)
        
        # Thruster configuration (same as controller)
        self.FORWARD_THRUSTERS = [4, 5]  # +Y direction
        self.BACK_THRUSTERS = [6, 7]     # -Y direction
        self.LEFT_THRUSTERS = [0, 1]     # +X direction  
        self.RIGHT_THRUSTERS = [2, 3]    # -X direction
        
        # Physics update timer
        self.physics_timer = self.create_timer(0.05, self.update_physics)  # 20 Hz
        
        # Velocity publishing timer
        self.velocity_timer = self.create_timer(0.1, self.publish_velocity)  # 10 Hz
        
        self.get_logger().info('Velocity Publisher Started')
        self.get_logger().info('Mode: Mock physics simulation (will switch to real if SRB data available)')
        
    def thrust_callback(self, msg):
        """Handle thrust commands"""
        if len(msg.data) >= 8:
            self.last_thrust_command = np.array(msg.data[:8])
            
    def joint_states_callback(self, msg):
        """Handle joint states - might contain real velocity data"""
        # If we get real velocity data from SRB, use it instead of mock
        if len(msg.velocity) >= 3:
            self.velocity[0] = msg.velocity[0] if len(msg.velocity) > 0 else 0.0
            self.velocity[1] = msg.velocity[1] if len(msg.velocity) > 1 else 0.0
            self.velocity[2] = msg.velocity[2] if len(msg.velocity) > 2 else 0.0
            
            self.get_logger().info('Using real velocity data from SRB joint states', throttle_duration_sec=5.0)
            
    def update_physics(self):
        """Update mock physics simulation"""
        dt = 0.05  # 20 Hz update rate
        
        # Calculate net thrust forces from thruster commands
        thrust_force = np.array([0.0, 0.0, 0.0])
        
        # X-axis thrust (left/right)
        left_thrust = sum(self.last_thrust_command[i] for i in self.LEFT_THRUSTERS)
        right_thrust = sum(self.last_thrust_command[i] for i in self.RIGHT_THRUSTERS)
        thrust_force[0] = (left_thrust - right_thrust) * self.thrust_scale
        
        # Y-axis thrust (forward/back)
        forward_thrust = sum(self.last_thrust_command[i] for i in self.FORWARD_THRUSTERS)
        back_thrust = sum(self.last_thrust_command[i] for i in self.BACK_THRUSTERS)
        thrust_force[1] = (forward_thrust - back_thrust) * self.thrust_scale
        
        # Z-axis: no thrusters configured
        thrust_force[2] = 0.0
        
        # Calculate acceleration: F = ma -> a = F/m
        acceleration = thrust_force / self.spacecraft_mass
        
        # Update velocity with physics integration
        # v = v * drag + a * dt (include drag for stability)
        self.velocity = self.velocity * self.drag_coefficient + acceleration * dt
        
        # Add some noise/disturbances for realism (small random forces)
        disturbance = np.random.normal(0, 0.001, 3)  # Small random accelerations
        self.velocity += disturbance * dt
        
    def publish_velocity(self):
        """Publish current velocity"""
        msg = Vector3()
        msg.x = float(self.velocity[0])
        msg.y = float(self.velocity[1])
        msg.z = float(self.velocity[2])
        
        self.velocity_pub.publish(msg)
        
        # Log velocity periodically for debugging
        velocity_magnitude = np.linalg.norm(self.velocity)
        if velocity_magnitude > 0.001:  # Only log when moving
            self.get_logger().debug(
                f'Velocity: [{self.velocity[0]:.4f}, {self.velocity[1]:.4f}, {self.velocity[2]:.4f}] '
                f'Magnitude: {velocity_magnitude:.4f} m/s'
            )


def main():
    rclpy.init()
    
    try:
        publisher = VelocityPublisher()
        
        print("=" * 50)
        print("SPACECRAFT VELOCITY PUBLISHER")
        print("=" * 50)
        print("Publishing spacecraft velocity to: /spacecraft/velocity")
        print("Subscribing to thrust commands: /srb/env0/robot/thrust")
        print("Mode: Mock physics simulation")
        print("\nThis provides velocity feedback for the PID controller.")
        print("The velocity is calculated using simplified physics based on")
        print("the thrust commands sent to the spacecraft.")
        print("=" * 50)
        
        rclpy.spin(publisher)
        
    except KeyboardInterrupt:
        print('\nShutting down velocity publisher...')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
