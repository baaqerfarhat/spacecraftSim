#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import json
import sys


class MovementClient(Node):
    def __init__(self):
        super().__init__('movement_client')
        
        self.command_pub = self.create_publisher(
            String,
            '/spacecraft/movement/command',
            10
        )
        
        self.emergency_pub = self.create_publisher(
            Bool,
            '/spacecraft/emergency_stop',
            10
        )
        
        self.status_sub = self.create_subscription(
            String,
            '/spacecraft/movement/status',
            self.status_callback,
            10
        )
        
        self.completion_sub = self.create_subscription(
            Bool,
            '/spacecraft/movement/completed',
            self.completion_callback,
            10
        )
        
        self.get_logger().info('Movement Client Ready')
        
    def status_callback(self, msg):
        """Display status updates"""
        print(f"[STATUS] {msg.data}")
        
    def completion_callback(self, msg):
        """Handle completion notifications"""
        if msg.data:
            print("[COMPLETED] Movement finished successfully!")
        
    def send_command(self, action, params=None):
        """Send movement command"""
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
        
    def emergency_stop(self, activate=True):
        """Send emergency stop command"""
        msg = Bool()
        msg.data = activate
        self.emergency_pub.publish(msg)
        
        status = "ACTIVATED" if activate else "DEACTIVATED"
        print(f"[EMERGENCY] Stop {status}")
        
    def controlled_movement(self, power=0.1, time_duration=1.5):
        """Execute controlled movement"""
        self.send_command("controlled_movement", {
            "power": power,
            "time": time_duration
        })
        
    def gentle_movement(self, pulse_power=0.05, num_pulses=3, pulse_time=0.3):
        """Execute gentle pulsed movement"""
        self.send_command("gentle_movement", {
            "pulse_power": pulse_power,
            "num_pulses": num_pulses,
            "pulse_time": pulse_time
        })
        
    def precise_movement(self, power=0.08, forward_time=0.5, brake_time=0.4):
        """Execute precise movement"""
        self.send_command("precise_movement", {
            "power": power,
            "forward_time": forward_time,
            "brake_time": brake_time
        })
        
    def configure(self, **params):
        """Configure controller parameters"""
        self.send_command("configure", params)
        
    def stop(self):
        """Stop current movement"""
        self.send_command("stop")


def print_help():
    """Print usage help"""
    print("\n" + "="*60)
    print("SPACECRAFT MOVEMENT CLIENT")
    print("="*60)
    print("\nUsage: python3 movement_client.py <command> [options]")
    print("\nCommands:")
    print("  controlled [power] [time]    - Controlled movement (default: 0.1, 1.5)")
    print("  gentle [power] [pulses]      - Gentle pulsed movement (default: 0.05, 3)")
    print("  precise [power] [fwd] [brk]  - Precise movement (default: 0.08, 0.5, 0.4)")
    print("  stop                         - Stop current movement")
    print("  emergency                    - Emergency stop")
    print("  resume                       - Resume from emergency stop")
    print("  configure [param=value]      - Configure parameters")
    print("  monitor                      - Monitor status (press Ctrl+C to exit)")
    print("\nExamples:")
    print("  python3 movement_client.py controlled 0.15 2.0")
    print("  python3 movement_client.py gentle 0.03 5")
    print("  python3 movement_client.py configure default_power=0.12")
    print("  python3 movement_client.py monitor")
    print("="*60)


def main():
    if len(sys.argv) < 2:
        print_help()
        return
        
    rclpy.init()
    
    try:
        client = MovementClient()
        
        # Give the node time to initialize
        import time
        time.sleep(0.5)
        
        command = sys.argv[1].lower()
        
        if command == "controlled":
            power = float(sys.argv[2]) if len(sys.argv) > 2 else 0.1
            time_duration = float(sys.argv[3]) if len(sys.argv) > 3 else 1.5
            client.controlled_movement(power, time_duration)
            print(f"Executing controlled movement: power={power}, time={time_duration}")
            
        elif command == "gentle":
            power = float(sys.argv[2]) if len(sys.argv) > 2 else 0.05
            pulses = int(sys.argv[3]) if len(sys.argv) > 3 else 3
            client.gentle_movement(power, pulses)
            print(f"Executing gentle movement: power={power}, pulses={pulses}")
            
        elif command == "precise":
            power = float(sys.argv[2]) if len(sys.argv) > 2 else 0.08
            fwd_time = float(sys.argv[3]) if len(sys.argv) > 3 else 0.5
            brk_time = float(sys.argv[4]) if len(sys.argv) > 4 else 0.4
            client.precise_movement(power, fwd_time, brk_time)
            print(f"Executing precise movement: power={power}, forward={fwd_time}, brake={brk_time}")
            
        elif command == "stop":
            client.stop()
            print("Stopping movement")
            
        elif command == "emergency":
            client.emergency_stop(True)
            print("Emergency stop activated")
            
        elif command == "resume":
            client.emergency_stop(False)
            print("Emergency stop deactivated")
            
        elif command == "configure":
            params = {}
            for arg in sys.argv[2:]:
                if "=" in arg:
                    key, value = arg.split("=", 1)
                    try:
                        # Try to convert to float, fallback to string
                        params[key] = float(value)
                    except ValueError:
                        params[key] = value
            client.configure(**params)
            print(f"Configuring parameters: {params}")
            
        elif command == "monitor":
            print("Monitoring spacecraft status... (Press Ctrl+C to exit)")
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
