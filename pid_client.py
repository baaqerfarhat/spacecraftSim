#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Vector3
import json
import sys
import time


class PIDClient(Node):
    def __init__(self):
        super().__init__('pid_client')
        
        self.command_pub = self.create_publisher(
            String,
            '/spacecraft/pid/command',
            10
        )
        
        self.setpoint_pub = self.create_publisher(
            Vector3,
            '/spacecraft/pid/setpoint',
            10
        )
        
        self.emergency_pub = self.create_publisher(
            Bool,
            '/spacecraft/emergency_stop',
            10
        )
        
        self.status_sub = self.create_subscription(
            String,
            '/spacecraft/pid/status',
            self.status_callback,
            10
        )
        
        self.velocity_sub = self.create_subscription(
            Vector3,
            '/spacecraft/pid/velocity',
            self.velocity_callback,
            10
        )
        
        self.get_logger().info('PID Client Ready')
        
    def status_callback(self, msg):
        """Display status updates"""
        print(f"[STATUS] {msg.data}")
        
    def velocity_callback(self, msg):
        """Display velocity updates"""
        velocity_mag = (msg.x**2 + msg.y**2 + msg.z**2)**0.5
        print(f"[VELOCITY] X: {msg.x:.4f}, Y: {msg.y:.4f}, Z: {msg.z:.4f} | Magnitude: {velocity_mag:.4f} m/s")
        
    def send_command(self, action, params=None):
        """Send PID command"""
        if params is None:
            params = {}
            
        command = {
            "action": action,
            "params": params
        }
        
        msg = String()
        msg.data = json.dumps(command)
        self.command_pub.publish(msg)
        
        print(f"[COMMAND] Sent: {action} with params: {params}")
        
    def set_velocity_target(self, x=0.0, y=0.0, z=0.0):
        """Set velocity target"""
        msg = Vector3()
        msg.x = float(x)
        msg.y = float(y)
        msg.z = float(z)
        self.setpoint_pub.publish(msg)
        
        print(f"[SETPOINT] Target velocity: [{x:.3f}, {y:.3f}, {z:.3f}]")
        
    def emergency_stop(self, activate=True):
        """Send emergency stop command"""
        msg = Bool()
        msg.data = activate
        self.emergency_pub.publish(msg)
        
        status = "ACTIVATED" if activate else "DEACTIVATED"
        print(f"[EMERGENCY] Stop {status}")
        
    def move_and_stop(self, power=0.1, duration=1.5):
        """Execute move and automatic PID stop"""
        self.send_command("move_and_stop", {
            "power": power,
            "duration": duration
        })
        
    def start_pid_control(self):
        """Start PID control to zero velocity"""
        self.send_command("move_to_zero")
        
    def stop_pid_control(self):
        """Stop PID control"""
        self.send_command("stop")
        
    def tune_pid(self, **gains):
        """Tune PID gains"""
        self.send_command("tune_pid", gains)


def print_help():
    """Print usage help"""
    print("\n" + "="*60)
    print("SPACECRAFT PID CONTROLLER CLIENT")
    print("="*60)
    print("\nUsage: python3 pid_client.py <command> [options]")
    print("\nCommands:")
    print("  move_stop [power] [duration]  - Move then PID stop (default: 0.1, 1.5)")
    print("  start                         - Start PID control to zero velocity")
    print("  stop                          - Stop PID control")
    print("  setpoint [x] [y] [z]         - Set velocity target (default: 0,0,0)")
    print("  emergency                     - Emergency stop")
    print("  resume                        - Resume from emergency stop")
    print("  tune [axis_param=value]       - Tune PID gains")
    print("  monitor                       - Monitor status and velocity")
    print("\nExamples:")
    print("  python3 pid_client.py move_stop 0.12 2.0")
    print("  python3 pid_client.py start")
    print("  python3 pid_client.py setpoint 0.0 0.5 0.0")
    print("  python3 pid_client.py tune y_kp=1.2 y_kd=0.4")
    print("  python3 pid_client.py monitor")
    print("\nPID Tuning Parameters:")
    print("  x_kp, x_ki, x_kd  - X-axis gains")
    print("  y_kp, y_ki, y_kd  - Y-axis gains") 
    print("  z_kp, z_ki, z_kd  - Z-axis gains")
    print("="*60)


def main():
    if len(sys.argv) < 2:
        print_help()
        return
        
    rclpy.init()
    
    try:
        client = PIDClient()
        
        # Give the node time to initialize
        time.sleep(0.5)
        
        command = sys.argv[1].lower()
        
        if command == "move_stop":
            power = float(sys.argv[2]) if len(sys.argv) > 2 else 0.1
            duration = float(sys.argv[3]) if len(sys.argv) > 3 else 1.5
            client.move_and_stop(power, duration)
            print(f"Executing move and stop: power={power}, duration={duration}")
            
        elif command == "start":
            client.start_pid_control()
            print("Starting PID control to zero velocity")
            
        elif command == "stop":
            client.stop_pid_control()
            print("Stopping PID control")
            
        elif command == "setpoint":
            x = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
            y = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
            z = float(sys.argv[4]) if len(sys.argv) > 4 else 0.0
            client.set_velocity_target(x, y, z)
            print(f"Setting velocity target: [{x:.3f}, {y:.3f}, {z:.3f}]")
            
        elif command == "emergency":
            client.emergency_stop(True)
            print("Emergency stop activated")
            
        elif command == "resume":
            client.emergency_stop(False)
            print("Emergency stop deactivated")
            
        elif command == "tune":
            gains = {}
            for arg in sys.argv[2:]:
                if "=" in arg:
                    key, value = arg.split("=", 1)
                    try:
                        gains[key] = float(value)
                    except ValueError:
                        print(f"Invalid gain value: {arg}")
                        return
            client.tune_pid(**gains)
            print(f"Tuning PID gains: {gains}")
            
        elif command == "monitor":
            print("Monitoring spacecraft PID control... (Press Ctrl+C to exit)")
            print("Status and velocity updates will appear below:")
            print("-" * 60)
            rclpy.spin(client)
            
        else:
            print(f"Unknown command: {command}")
            print_help()
            return
            
        # For non-monitor commands, wait briefly for the command to be sent
        if command != "monitor":
            time.sleep(1.0)
            
    except KeyboardInterrupt:
        print("\nExiting...")
    except Exception as e:
        print(f"Error: {e}")
        print_help()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
