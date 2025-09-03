import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import time


class ManualController(Node):
    def __init__(self):
        super().__init__('manual_controller')
        
        # Create publisher for thruster commands
        self.thruster_pub = self.create_publisher(
            Float32MultiArray, 
            '/srb/env0/robot/thrust', 
            10
        )
        
        # Thruster configuration
        self.RIGHT_THRUSTERS = [0, 1]  # +X direction (right)
        self.LEFT_THRUSTERS = [2, 3]   # -X direction (left) 
        
        self.get_logger().info('Manual Controller initialized')
        self.get_logger().info('Thruster layout:')
        self.get_logger().info('  Right (0,1): +X direction')
        self.get_logger().info('  Left (2,3): -X direction') 
        
    def send_command(self, thrust_values, description=''):
        msg = Float32MultiArray()
        msg.data = thrust_values
        self.thruster_pub.publish(msg)
        self.get_logger().info(f'{description}: {thrust_values}')
        
    def stop(self):
        self.send_command([0.0] * 8, 'STOP')
        
    def move_right(self, power=0.3):
        cmd = [0.0] * 8
        for i in self.RIGHT_THRUSTERS:
            cmd[i] = power
        self.send_command(cmd, f'MOVE RIGHT ({power})')
        
    def move_left(self, power=0.3):
        cmd = [0.0] * 8
        for i in self.LEFT_THRUSTERS:
            cmd[i] = power
        self.send_command(cmd, f'MOVE LEFT ({power})')
        
    def run_movement_sequence(self):
        try:
            print('=== CONTROLLED RIGHT MOVEMENT TEST ===')
            
            # Move right for 3 seconds
            print('1. Moving RIGHT for 3 seconds...')
            self.move_right(0.3)
            time.sleep(3.0)
            
            # Brake for 2 seconds
            print('2. Braking (LEFT thrust) for 2 seconds...')
            self.move_left(0.3)
            time.sleep(2.0)
            
            # Stop
            print('3. Stopping...')
            self.stop()
            time.sleep(1.0)
            
            print('=== MOVEMENT COMPLETE ===')
            
        except Exception as e:
            self.get_logger().error(f'Error during movement: {e}')
            self.stop()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        controller = ManualController()
        time.sleep(1.0)  # Wait for publisher
        
        # Run the movement sequence
        controller.run_movement_sequence()
        
        # Keep node alive briefly
        time.sleep(1.0)
        
    except KeyboardInterrupt:
        print('Interrupted by user')
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if 'controller' in locals():
            controller.stop()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
