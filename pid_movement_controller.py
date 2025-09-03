#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String, Bool
from geometry_msgs.msg import Vector3, Twist
from sensor_msgs.msg import JointState
import time
import threading
import json
import numpy as np


class PIDController:
    """PID Controller for spacecraft velocity control"""
    def __init__(self, kp=1.0, ki=0.0, kd=0.1, max_output=1.0, integral_windup_limit=10.0):
        self.kp = kp
        self.ki = ki  
        self.kd = kd
        self.max_output = max_output
        self.integral_windup_limit = integral_windup_limit
        
        # State variables
        self.integral = 0.0
        self.previous_error = 0.0
        self.last_time = None
        
        self.get_logger().info(f'PID Controller: Kp={kp}, Ki={ki}, Kd={kd}')
        
    def reset(self):
        """Reset controller state"""
        self.integral = 0.0
        self.previous_error = 0.0
        self.last_time = None
        
    def update(self, error, current_time):
        """
        Update PID controller with current error
        
        Args:
            error: Current error (desired - actual)
            current_time: Current time in seconds
            
        Returns:
            Control output
        """
        if self.last_time is None:
            self.last_time = current_time
            self.previous_error = error
            return self.kp * error
            
        dt = current_time - self.last_time
        
        if dt <= 0:
            return self.kp * error
            
        # Proportional term
        P = self.kp * error
        
        # Integral term with windup protection
        self.integral += error * dt
        # Anti-windup: clamp integral term
        if abs(self.integral) > self.integral_windup_limit:
            self.integral = np.sign(self.integral) * self.integral_windup_limit
        I = self.ki * self.integral
        
        # Derivative term
        derivative = (error - self.previous_error) / dt
        D = self.kd * derivative
        
        # Calculate total output
        output = P + I + D
        
        # Clamp output
        output = np.clip(output, -self.max_output, self.max_output)
        
        # Update state
        self.previous_error = error
        self.last_time = current_time
        
        return output


class SpacecraftPIDController(Node):
    def __init__(self):
        super().__init__('spacecraft_pid_controller')
        
        # Publishers
        self.thruster_pub = self.create_publisher(
            Float32MultiArray, 
            '/srb/env0/robot/thrust', 
            10
        )
        
        self.status_pub = self.create_publisher(
            String,
            '/spacecraft/pid/status',
            10
        )
        
        self.velocity_pub = self.create_publisher(
            Vector3,
            '/spacecraft/pid/velocity',
            10
        )
        
        # Subscribers - Try multiple possible velocity topics
        self.velocity_sub = self.create_subscription(
            Vector3,
            '/spacecraft/velocity',  # Custom topic we'll need to create
            self.velocity_callback,
            10
        )
        
        # Subscribe to joint states (might contain velocity info)
        self.joint_states_sub = self.create_subscription(
            JointState,
            '/srb/env0/robot/joint_states',
            self.joint_states_callback,
            10
        )
        
        # Control subscribers
        self.command_sub = self.create_subscription(
            String,
            '/spacecraft/pid/command',
            self.command_callback,
            10
        )
        
        self.setpoint_sub = self.create_subscription(
            Vector3,
            '/spacecraft/pid/setpoint',
            self.setpoint_callback,
            10
        )
        
        self.emergency_stop_sub = self.create_subscription(
            Bool,
            '/spacecraft/emergency_stop',
            self.emergency_stop_callback,
            10
        )
        
        # Thruster configuration
        self.FORWARD_THRUSTERS = [4, 5]  # +Y direction
        self.BACK_THRUSTERS = [6, 7]     # -Y direction
        self.LEFT_THRUSTERS = [0, 1]     # +X direction  
        self.RIGHT_THRUSTERS = [2, 3]    # -X direction
        
        # PID Controllers for each axis
        self.pid_x = PIDController(kp=0.8, ki=0.1, kd=0.3, max_output=0.5)
        self.pid_y = PIDController(kp=0.8, ki=0.1, kd=0.3, max_output=0.5)
        self.pid_z = PIDController(kp=0.4, ki=0.05, kd=0.2, max_output=0.3)  # Lower gains for Z
        
        # State
        self.current_velocity = np.array([0.0, 0.0, 0.0])  # [x, y, z]
        self.target_velocity = np.array([0.0, 0.0, 0.0])   # Default: stop
        self.controller_active = False
        self.emergency_stop_active = False
        
        # Control loop
        self.control_thread = None
        self.control_rate = 20.0  # Hz
        self.velocity_tolerance = 0.01  # m/s - consider stopped if below this
        
        # Mock velocity for testing (remove when real velocity topic is available)
        self.mock_velocity_timer = self.create_timer(0.1, self.publish_mock_velocity)
        self.mock_vel = np.array([0.0, 0.0, 0.0])
        self.last_thrust_command = np.array([0.0] * 8)
        
        self.get_logger().info('Spacecraft PID Controller Started')
        self.get_logger().info('PID Gains:')
        self.get_logger().info(f'  X-axis: Kp={self.pid_x.kp}, Ki={self.pid_x.ki}, Kd={self.pid_x.kd}')
        self.get_logger().info(f'  Y-axis: Kp={self.pid_y.kp}, Ki={self.pid_y.ki}, Kd={self.pid_y.kd}')
        self.get_logger().info(f'  Z-axis: Kp={self.pid_z.kp}, Ki={self.pid_z.ki}, Kd={self.pid_z.kd}')
        
        self.publish_status("READY")
        
    def publish_mock_velocity(self):
        """Temporary: Simulate velocity feedback based on thrust commands"""
        # Simple physics simulation for testing
        dt = 0.1
        mass = 100.0  # kg (approximate spacecraft mass)
        drag = 0.95   # Drag coefficient (space has no air resistance, but add for stability)
        
        # Calculate thrust forces
        thrust_force = np.array([0.0, 0.0, 0.0])
        
        # X-axis thrusters (left/right)
        thrust_force[0] = (sum(self.last_thrust_command[i] for i in self.LEFT_THRUSTERS) - 
                          sum(self.last_thrust_command[i] for i in self.RIGHT_THRUSTERS))
        
        # Y-axis thrusters (forward/back)  
        thrust_force[1] = (sum(self.last_thrust_command[i] for i in self.FORWARD_THRUSTERS) - 
                          sum(self.last_thrust_command[i] for i in self.BACK_THRUSTERS))
        
        # Z-axis: not implemented for this simple spacecraft
        thrust_force[2] = 0.0
        
        # Update mock velocity with simple physics
        acceleration = thrust_force * 1000.0 / mass  # Scale thrust to reasonable force
        self.mock_vel = self.mock_vel * drag + acceleration * dt
        
        # Publish as velocity feedback
        vel_msg = Vector3()
        vel_msg.x = float(self.mock_vel[0])
        vel_msg.y = float(self.mock_vel[1])
        vel_msg.z = float(self.mock_vel[2])
        
        # Simulate receiving this as feedback
        self.velocity_callback(vel_msg)
        
    def velocity_callback(self, msg):
        """Handle velocity feedback"""
        self.current_velocity[0] = msg.x
        self.current_velocity[1] = msg.y
        self.current_velocity[2] = msg.z
        
        # Republish for monitoring
        self.velocity_pub.publish(msg)
        
    def joint_states_callback(self, msg):
        """Handle joint states (might contain velocity info)"""
        # Joint states might have velocity information
        if len(msg.velocity) >= 3:
            # Use first 3 velocity values as body velocities
            self.current_velocity[0] = msg.velocity[0] if len(msg.velocity) > 0 else 0.0
            self.current_velocity[1] = msg.velocity[1] if len(msg.velocity) > 1 else 0.0
            self.current_velocity[2] = msg.velocity[2] if len(msg.velocity) > 2 else 0.0
            
    def setpoint_callback(self, msg):
        """Set new velocity target"""
        self.target_velocity[0] = msg.x
        self.target_velocity[1] = msg.y
        self.target_velocity[2] = msg.z
        self.get_logger().info(f'New velocity setpoint: [{msg.x:.3f}, {msg.y:.3f}, {msg.z:.3f}]')
        
    def emergency_stop_callback(self, msg):
        """Handle emergency stop"""
        self.emergency_stop_active = msg.data
        if self.emergency_stop_active:
            self.get_logger().warn('EMERGENCY STOP ACTIVATED')
            self.stop_control()
            self.send_thrust_command([0.0] * 8)
            self.publish_status("EMERGENCY_STOP")
        else:
            self.get_logger().info('Emergency stop deactivated')
            self.publish_status("READY")
            
    def command_callback(self, msg):
        """Handle control commands"""
        try:
            command = json.loads(msg.data)
            action = command.get('action', '').lower()
            
            if action == 'start':
                # Start PID control to current target
                self.start_control()
                
            elif action == 'stop':
                # Stop PID control
                self.stop_control()
                
            elif action == 'move_to_zero':
                # Set target to zero and start control
                self.target_velocity = np.array([0.0, 0.0, 0.0])
                self.start_control()
                
            elif action == 'move_and_stop':
                # Execute a move then automatically stop
                params = command.get('params', {})
                power = params.get('power', 0.1)
                duration = params.get('duration', 1.0)
                self.execute_move_and_stop(power, duration)
                
            elif action == 'tune_pid':
                # Update PID gains
                params = command.get('params', {})
                self.tune_pid_gains(params)
                
            else:
                self.get_logger().error(f'Unknown action: {action}')
                
        except json.JSONDecodeError:
            self.get_logger().error('Invalid JSON command')
        except Exception as e:
            self.get_logger().error(f'Command execution error: {str(e)}')
            
    def tune_pid_gains(self, params):
        """Update PID controller gains"""
        for axis, pid in [('x', self.pid_x), ('y', self.pid_y), ('z', self.pid_z)]:
            if f'{axis}_kp' in params:
                pid.kp = float(params[f'{axis}_kp'])
            if f'{axis}_ki' in params:
                pid.ki = float(params[f'{axis}_ki'])
            if f'{axis}_kd' in params:
                pid.kd = float(params[f'{axis}_kd'])
                
        self.get_logger().info('PID gains updated')
        
    def start_control(self):
        """Start PID control loop"""
        if self.emergency_stop_active:
            self.get_logger().warn('Cannot start control: Emergency stop active')
            return
            
        if self.controller_active:
            self.get_logger().warn('Control already active')
            return
            
        self.controller_active = True
        
        # Reset PID controllers
        self.pid_x.reset()
        self.pid_y.reset()
        self.pid_z.reset()
        
        # Start control thread
        self.control_thread = threading.Thread(target=self._control_loop)
        self.control_thread.daemon = True
        self.control_thread.start()
        
        self.get_logger().info('PID control started')
        self.publish_status("CONTROLLING")
        
    def stop_control(self):
        """Stop PID control loop"""
        self.controller_active = False
        if self.control_thread and self.control_thread.is_alive():
            self.control_thread.join(timeout=1.0)
            
        # Send zero thrust
        self.send_thrust_command([0.0] * 8)
        self.get_logger().info('PID control stopped')
        self.publish_status("STOPPED")
        
    def _control_loop(self):
        """Main PID control loop"""
        rate = 1.0 / self.control_rate
        
        while self.controller_active and not self.emergency_stop_active:
            current_time = time.time()
            
            # Calculate velocity errors
            error_x = self.target_velocity[0] - self.current_velocity[0]
            error_y = self.target_velocity[1] - self.current_velocity[1]
            error_z = self.target_velocity[2] - self.current_velocity[2]
            
            # Calculate PID outputs
            output_x = self.pid_x.update(error_x, current_time)
            output_y = self.pid_y.update(error_y, current_time)
            output_z = self.pid_z.update(error_z, current_time)
            
            # Convert PID outputs to thruster commands
            thrust_cmd = self.pid_to_thrust(output_x, output_y, output_z)
            
            # Send thrust command
            self.send_thrust_command(thrust_cmd)
            
            # Check if we've reached the target
            velocity_magnitude = np.linalg.norm(self.current_velocity - self.target_velocity)
            if velocity_magnitude < self.velocity_tolerance:
                self.get_logger().info(f'Target reached! Velocity error: {velocity_magnitude:.4f} m/s')
                if np.allclose(self.target_velocity, [0.0, 0.0, 0.0]):
                    # If target was zero, we've successfully stopped
                    self.stop_control()
                    self.publish_status("TARGET_REACHED")
                    break
                    
            # Log status periodically
            if int(current_time) % 2 == 0:  # Every 2 seconds
                self.get_logger().info(
                    f'Vel: [{self.current_velocity[0]:.3f}, {self.current_velocity[1]:.3f}, {self.current_velocity[2]:.3f}] '
                    f'Target: [{self.target_velocity[0]:.3f}, {self.target_velocity[1]:.3f}, {self.target_velocity[2]:.3f}] '
                    f'Error: {velocity_magnitude:.4f}'
                )
                
            time.sleep(rate)
            
    def pid_to_thrust(self, output_x, output_y, output_z):
        """Convert PID outputs to thruster commands"""
        thrust_cmd = [0.0] * 8
        
        # X-axis control (left/right thrusters)
        if output_x > 0:  # Need to move right, use left thrusters
            for i in self.LEFT_THRUSTERS:
                thrust_cmd[i] = abs(output_x)
        else:  # Need to move left, use right thrusters
            for i in self.RIGHT_THRUSTERS:
                thrust_cmd[i] = abs(output_x)
                
        # Y-axis control (forward/back thrusters)
        if output_y > 0:  # Need to move forward, use forward thrusters
            for i in self.FORWARD_THRUSTERS:
                thrust_cmd[i] = abs(output_y)
        else:  # Need to move back, use back thrusters
            for i in self.BACK_THRUSTERS:
                thrust_cmd[i] = abs(output_y)
                
        # Z-axis control not implemented for this spacecraft config
        
        return thrust_cmd
        
    def send_thrust_command(self, thrust_values):
        """Send thrust command to spacecraft"""
        if self.emergency_stop_active:
            thrust_values = [0.0] * 8
            
        msg = Float32MultiArray()
        msg.data = thrust_values
        self.thruster_pub.publish(msg)
        
        # Store for mock velocity simulation
        self.last_thrust_command = np.array(thrust_values)
        
    def execute_move_and_stop(self, power, duration):
        """Execute a movement then automatically engage PID to stop"""
        def move_and_stop_worker():
            try:
                self.get_logger().info(f'Executing move: {power*100}% power for {duration}s, then PID stop')
                self.publish_status("MOVING")
                
                # Phase 1: Apply forward thrust
                forward_thrust = [0.0] * 8
                for i in self.FORWARD_THRUSTERS:
                    forward_thrust[i] = power
                    
                self.send_thrust_command(forward_thrust)
                time.sleep(duration)
                
                # Phase 2: Stop thrusters and engage PID to bring to zero
                self.send_thrust_command([0.0] * 8)
                time.sleep(0.1)  # Brief pause
                
                # Phase 3: Engage PID control to drive velocity to zero
                self.target_velocity = np.array([0.0, 0.0, 0.0])
                self.start_control()
                
            except Exception as e:
                self.get_logger().error(f'Move and stop error: {str(e)}')
                self.stop_control()
                
        # Execute in separate thread
        thread = threading.Thread(target=move_and_stop_worker)
        thread.daemon = True
        thread.start()
        
    def publish_status(self, status):
        """Publish current status"""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)


def main():
    rclpy.init()
    
    try:
        controller = SpacecraftPIDController()
        
        print("=" * 60)
        print("SPACECRAFT PID CONTROLLER READY")
        print("=" * 60)
        print("\nUsage Examples:")
        print("\n1. Move and automatically stop:")
        print('   ros2 topic pub /spacecraft/pid/command std_msgs/String \'{"action": "move_and_stop", "params": {"power": 0.1, "duration": 1.5}}\'')
        print("\n2. Start PID control to zero velocity:")
        print('   ros2 topic pub /spacecraft/pid/command std_msgs/String \'{"action": "move_to_zero"}\'')
        print("\n3. Set velocity target:")
        print('   ros2 topic pub /spacecraft/pid/setpoint geometry_msgs/Vector3 \'x: 0.0, y: 0.0, z: 0.0\'')
        print("\n4. Tune PID gains:")
        print('   ros2 topic pub /spacecraft/pid/command std_msgs/String \'{"action": "tune_pid", "params": {"y_kp": 1.2, "y_kd": 0.4}}\'')
        print("\n5. Emergency stop:")
        print('   ros2 topic pub /spacecraft/emergency_stop std_msgs/Bool \'data: true\'')
        print("\nMonitor:")
        print('   ros2 topic echo /spacecraft/pid/status')
        print('   ros2 topic echo /spacecraft/pid/velocity')
        print("=" * 60)
        
        rclpy.spin(controller)
        
    except KeyboardInterrupt:
        print('\nShutting down PID controller...')
    finally:
        if 'controller' in locals():
            controller.stop_control()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
