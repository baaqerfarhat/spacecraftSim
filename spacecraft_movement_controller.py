#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String, Bool
from geometry_msgs.msg import Twist
import time
import threading
import json


class SpacecraftMovementController(Node):
    def __init__(self):
        super().__init__('spacecraft_movement_controller')
        
        # Publishers
        self.thruster_pub = self.create_publisher(
            Float32MultiArray, 
            '/srb/env0/robot/thrust', 
            10
        )
        
        self.status_pub = self.create_publisher(
            String,
            '/spacecraft/movement/status',
            10
        )
        
        self.completion_pub = self.create_publisher(
            Bool,
            '/spacecraft/movement/completed',
            10
        )
        
        # Subscribers
        self.command_sub = self.create_subscription(
            String,
            '/spacecraft/movement/command',
            self.command_callback,
            10
        )
        
        self.twist_sub = self.create_subscription(
            Twist,
            '/spacecraft/cmd_vel',
            self.twist_callback,
            10
        )
        
        self.emergency_stop_sub = self.create_subscription(
            Bool,
            '/spacecraft/emergency_stop',
            self.emergency_stop_callback,
            10
        )
        
        # Movement configuration
        self.FORWARD_THRUSTERS = [4, 5]  # +Y direction
        self.BACK_THRUSTERS = [6, 7]     # -Y direction
        
        # State management
        self.is_moving = False
        self.emergency_stop_active = False
        self.movement_thread = None
        
        # Movement parameters (can be configured via commands)
        self.default_power = 0.1
        self.default_time = 1.5
        self.cleanup_power = 0.03
        self.cleanup_time = 0.2
        
        self.get_logger().info('Spacecraft Movement Controller Started')
        self.get_logger().info('Available topics:')
        self.get_logger().info('  Commands: /spacecraft/movement/command')
        self.get_logger().info('  Twist: /spacecraft/cmd_vel')
        self.get_logger().info('  Emergency Stop: /spacecraft/emergency_stop')
        self.get_logger().info('  Status: /spacecraft/movement/status')
        self.get_logger().info('  Completion: /spacecraft/movement/completed')
        
        self.publish_status("READY")
        
    def publish_status(self, status):
        """Publish current status"""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
        self.get_logger().info(f'Status: {status}')
        
    def publish_completion(self, completed):
        """Publish movement completion status"""
        msg = Bool()
        msg.data = completed
        self.completion_pub.publish(msg)
        
    def send_command(self, thrust_values):
        """Send thrust command to spacecraft"""
        if self.emergency_stop_active:
            thrust_values = [0.0] * 8
            
        msg = Float32MultiArray()
        msg.data = thrust_values
        self.thruster_pub.publish(msg)
        
    def stop(self):
        """Stop all thrusters"""
        self.send_command([0.0] * 8)
        
    def move_forward(self, power=None):
        """Move forward with specified power"""
        if power is None:
            power = self.default_power
        cmd = [0.0] * 8
        for i in self.FORWARD_THRUSTERS:
            cmd[i] = power
        self.send_command(cmd)
        
    def move_back(self, power=None):
        """Move backward with specified power"""
        if power is None:
            power = self.default_power
        cmd = [0.0] * 8
        for i in self.BACK_THRUSTERS:
            cmd[i] = power
        self.send_command(cmd)
        
    def emergency_stop_callback(self, msg):
        """Handle emergency stop commands"""
        self.emergency_stop_active = msg.data
        if self.emergency_stop_active:
            self.get_logger().warn('EMERGENCY STOP ACTIVATED')
            self.stop()
            self.is_moving = False
            self.publish_status("EMERGENCY_STOP")
        else:
            self.get_logger().info('Emergency stop deactivated')
            self.publish_status("READY")
            
    def twist_callback(self, msg):
        """Handle Twist commands for manual control"""
        if self.is_moving or self.emergency_stop_active:
            return
            
        # Use linear.x for forward/backward movement
        linear_x = msg.linear.x
        
        if abs(linear_x) < 0.01:  # Dead zone
            self.stop()
        elif linear_x > 0:
            # Scale the power based on twist command (0.0 to 1.0)
            power = min(abs(linear_x) * 0.2, 0.2)  # Max 20% power
            self.move_forward(power)
        else:
            power = min(abs(linear_x) * 0.2, 0.2)  # Max 20% power
            self.move_back(power)
            
    def command_callback(self, msg):
        """Handle movement commands"""
        if self.emergency_stop_active:
            self.get_logger().warn('Cannot execute command: Emergency stop active')
            return
            
        try:
            # Parse JSON command
            command = json.loads(msg.data)
            action = command.get('action', '').lower()
            
            if self.is_moving and action != 'stop':
                self.get_logger().warn('Movement already in progress')
                return
                
            if action == 'controlled_movement':
                params = command.get('params', {})
                power = params.get('power', self.default_power)
                time_duration = params.get('time', self.default_time)
                self.execute_controlled_movement(power, time_duration)
                
            elif action == 'gentle_movement':
                params = command.get('params', {})
                self.execute_gentle_movement(params)
                
            elif action == 'precise_movement':
                params = command.get('params', {})
                self.execute_precise_movement(params)
                
            elif action == 'stop':
                self.stop()
                self.is_moving = False
                self.publish_status("STOPPED")
                
            elif action == 'configure':
                params = command.get('params', {})
                self.configure_parameters(params)
                
            else:
                self.get_logger().error(f'Unknown action: {action}')
                
        except json.JSONDecodeError:
            self.get_logger().error('Invalid JSON command')
        except Exception as e:
            self.get_logger().error(f'Command execution error: {str(e)}')
            
    def configure_parameters(self, params):
        """Configure movement parameters"""
        if 'default_power' in params:
            self.default_power = float(params['default_power'])
        if 'default_time' in params:
            self.default_time = float(params['default_time'])
        if 'cleanup_power' in params:
            self.cleanup_power = float(params['cleanup_power'])
        if 'cleanup_time' in params:
            self.cleanup_time = float(params['cleanup_time'])
            
        self.get_logger().info(f'Parameters updated: power={self.default_power}, time={self.default_time}')
        self.publish_status("CONFIGURED")
        
    def execute_controlled_movement(self, power=None, time_duration=None):
        """Execute controlled movement in separate thread"""
        if power is None:
            power = self.default_power
        if time_duration is None:
            time_duration = self.default_time
            
        self.movement_thread = threading.Thread(
            target=self._controlled_movement_worker,
            args=(power, time_duration)
        )
        self.movement_thread.start()
        
    def _controlled_movement_worker(self, power, time_duration):
        """Worker thread for controlled movement"""
        try:
            self.is_moving = True
            self.publish_status("MOVING_FORWARD")
            
            # Phase 1: Move forward
            self.get_logger().info(f'Phase 1: Moving FORWARD for {time_duration} seconds at {power*100}% power')
            self.move_forward(power)
            time.sleep(time_duration)
            
            if self.emergency_stop_active:
                return
                
            # Phase 2: Braking
            brake_time = time_duration * 0.95  # 5% less to avoid overshoot
            self.publish_status("BRAKING")
            self.get_logger().info(f'Phase 2: Braking for {brake_time:.2f} seconds')
            self.move_back(power)
            time.sleep(brake_time)
            
            if self.emergency_stop_active:
                return
                
            # Phase 3: Initial stop
            self.publish_status("STOPPING")
            self.stop()
            time.sleep(0.5)
            
            # Phase 4: Cleanup
            if not self.emergency_stop_active:
                self.publish_status("CLEANUP")
                self.get_logger().info('Phase 4: Cleanup pulses')
                for i in range(3):
                    if self.emergency_stop_active:
                        break
                    self.move_back(self.cleanup_power)
                    time.sleep(self.cleanup_time)
                    self.stop()
                    time.sleep(0.3)
            
            # Final stop
            self.stop()
            time.sleep(1.0)
            
            self.is_moving = False
            self.publish_status("COMPLETED")
            self.publish_completion(True)
            self.get_logger().info('=== CONTROLLED MOVEMENT COMPLETE ===')
            
        except Exception as e:
            self.get_logger().error(f'Movement error: {str(e)}')
            self.stop()
            self.is_moving = False
            self.publish_status("ERROR")
            
    def execute_gentle_movement(self, params):
        """Execute gentle pulsed movement"""
        self.movement_thread = threading.Thread(
            target=self._gentle_movement_worker,
            args=(params,)
        )
        self.movement_thread.start()
        
    def _gentle_movement_worker(self, params):
        """Worker thread for gentle movement"""
        try:
            self.is_moving = True
            pulse_time = params.get('pulse_time', 0.3)
            pulse_power = params.get('pulse_power', 0.05)
            num_pulses = params.get('num_pulses', 3)
            
            self.publish_status("GENTLE_MOVEMENT")
            self.get_logger().info('=== STARTING GENTLE PULSED MOVEMENT ===')
            
            # Forward pulses
            for i in range(num_pulses):
                if self.emergency_stop_active:
                    break
                self.get_logger().info(f'Forward pulse {i+1}/{num_pulses}')
                self.move_forward(pulse_power)
                time.sleep(pulse_time)
                self.stop()
                time.sleep(0.5)
            
            # Coast
            self.get_logger().info('Coasting...')
            time.sleep(2.0)
            
            # Brake pulses
            brake_pulses = num_pulses - 1
            for i in range(brake_pulses):
                if self.emergency_stop_active:
                    break
                self.get_logger().info(f'Brake pulse {i+1}/{brake_pulses}')
                self.move_back(pulse_power)
                time.sleep(pulse_time)
                self.stop()
                time.sleep(0.5)
                
            self.is_moving = False
            self.publish_status("COMPLETED")
            self.publish_completion(True)
            self.get_logger().info('=== GENTLE MOVEMENT COMPLETE ===')
            
        except Exception as e:
            self.get_logger().error(f'Gentle movement error: {str(e)}')
            self.stop()
            self.is_moving = False
            self.publish_status("ERROR")
            
    def execute_precise_movement(self, params):
        """Execute precise short movement"""
        self.movement_thread = threading.Thread(
            target=self._precise_movement_worker,
            args=(params,)
        )
        self.movement_thread.start()
        
    def _precise_movement_worker(self, params):
        """Worker thread for precise movement"""
        try:
            self.is_moving = True
            power = params.get('power', 0.08)
            forward_time = params.get('forward_time', 0.5)
            brake_time = params.get('brake_time', 0.4)
            
            self.publish_status("PRECISE_MOVEMENT")
            self.get_logger().info('=== STARTING PRECISE MOVEMENT ===')
            
            # Short forward
            self.move_forward(power)
            time.sleep(forward_time)
            self.stop()
            time.sleep(0.5)
            
            if not self.emergency_stop_active:
                # Short brake
                self.move_back(power)
                time.sleep(brake_time)
                self.stop()
                time.sleep(1.0)
            
            self.is_moving = False
            self.publish_status("COMPLETED")
            self.publish_completion(True)
            self.get_logger().info('=== PRECISE MOVEMENT COMPLETE ===')
            
        except Exception as e:
            self.get_logger().error(f'Precise movement error: {str(e)}')
            self.stop()
            self.is_moving = False
            self.publish_status("ERROR")


def main():
    rclpy.init()
    
    try:
        controller = SpacecraftMovementController()
        
        print("=" * 60)
        print("SPACECRAFT MOVEMENT CONTROLLER READY")
        print("=" * 60)
        print("\nUsage Examples:")
        print("\n1. Controlled Movement:")
        print('   ros2 topic pub /spacecraft/movement/command std_msgs/String \'{"action": "controlled_movement", "params": {"power": 0.1, "time": 2.0}}\'')
        print("\n2. Gentle Movement:")
        print('   ros2 topic pub /spacecraft/movement/command std_msgs/String \'{"action": "gentle_movement", "params": {"pulse_power": 0.05, "num_pulses": 3}}\'')
        print("\n3. Emergency Stop:")
        print('   ros2 topic pub /spacecraft/emergency_stop std_msgs/Bool \'data: true\'')
        print("\n4. Manual Control (use gamepad or keyboard):")
        print('   ros2 topic pub /spacecraft/cmd_vel geometry_msgs/Twist \'linear: {x: 0.5}\'')
        print("\n5. Configure Parameters:")
        print('   ros2 topic pub /spacecraft/movement/command std_msgs/String \'{"action": "configure", "params": {"default_power": 0.15}}\'')
        print("\nMonitor status:")
        print('   ros2 topic echo /spacecraft/movement/status')
        print("=" * 60)
        
        rclpy.spin(controller)
        
    except KeyboardInterrupt:
        print('\nShutting down controller...')
    finally:
        if 'controller' in locals():
            controller.stop()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
