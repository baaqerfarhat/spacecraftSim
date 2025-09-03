import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import time
import sys


class SpacecraftMover(Node):
    def __init__(self):
        super().__init__('spacecraft_mover')
        
        self.thruster_pub = self.create_publisher(
            Float32MultiArray, 
            '/srb/env0/robot/thrust', 
            10
        )
        
        # Thruster configuration based on your setup
        self.RIGHT_THRUSTERS = [0, 1]  # +X direction
        self.LEFT_THRUSTERS = [2, 3]   # -X direction
        
        self.get_logger().info('Spacecraft Mover Node Started')
        
    def send_command(self, thrust_values):
        msg = Float32MultiArray()
        msg.data = thrust_values
        self.thruster_pub.publish(msg)
        self.get_logger().info(f'Sent: {thrust_values}')
        
    def stop(self):
        self.send_command([0.0] * 8)
        
    def move_right(self, power=0.3):
        cmd = [0.0] * 8
        for i in self.RIGHT_THRUSTERS:
            cmd[i] = power
        self.send_command(cmd)
        
    def brake_right(self, power=0.3):
        cmd = [0.0] * 8
        for i in self.LEFT_THRUSTERS:
            cmd[i] = power
        self.send_command(cmd)
        
    def controlled_movement(self):
        print('=== STARTING CONTROLLED MOVEMENT ===')
        
        # Move right for 3 seconds
        print('Phase 1: Moving RIGHT for 3 seconds...')
        self.move_right(0.3)
        time.sleep(3.0)
        
        # Brake for 2 seconds
        print('Phase 2: Braking for 2 seconds...')
        self.brake_right(0.3)
        time.sleep(2.0)
        
        # Stop
        print('Phase 3: Full stop')
        self.stop()
        
        print('=== MOVEMENT COMPLETE ===')


def main():
    rclpy.init()
    
    try:
        mover = SpacecraftMover()
        time.sleep(1.0)  # Wait for publisher to connect
        
        mover.controlled_movement()
        
        time.sleep(1.0)
        
    except KeyboardInterrupt:
        print('Interrupted')
    finally:
        if 'mover' in locals():
            mover.stop()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
