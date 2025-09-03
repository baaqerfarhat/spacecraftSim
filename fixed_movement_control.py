import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import time


class FixedSpacecraftMover(Node):
    def __init__(self):
        super().__init__('fixed_spacecraft_mover')
        
        self.thruster_pub = self.create_publisher(
            Float32MultiArray, 
            '/srb/env0/robot/thrust', 
            10
        )
        
        # CORRECTED: Use forward/back thrusters for predictable movement
        self.FORWARD_THRUSTERS = [4, 5]  # +Y direction
        self.BACK_THRUSTERS = [6, 7]     # -Y direction
        
        self.get_logger().info('Fixed Spacecraft Mover Node Started')
        self.get_logger().info('Using FORWARD/BACK thrusters for controlled movement')
        
    def send_command(self, thrust_values):
        msg = Float32MultiArray()
        msg.data = thrust_values
        self.thruster_pub.publish(msg)
        self.get_logger().info(f'Sent: {thrust_values}')
        
    def stop(self):
        self.send_command([0.0] * 8)
        
    def move_forward(self, power=0.1):  # MUCH REDUCED power
        cmd = [0.0] * 8
        for i in self.FORWARD_THRUSTERS:
            cmd[i] = power
        self.send_command(cmd)
        
    def move_back(self, power=0.1):  # MUCH REDUCED power
        cmd = [0.0] * 8
        for i in self.BACK_THRUSTERS:
            cmd[i] = power
        self.send_command(cmd)
        
    def controlled_movement(self):
        print('=== STARTING CONTROLLED FORWARD MOVEMENT ===')
        
        # Phase 1: Move forward for 1.5 seconds (REDUCED time and power)
        print('Phase 1: Moving FORWARD for 1.5 seconds at 10% power...')
        self.move_forward(0.1)
        time.sleep(1.5)
        
        # Phase 2: Brake for 3 seconds (LONGER braking time)
        print('Phase 2: Braking for 3 seconds...')
        self.move_back(0.1)
        time.sleep(3.0)
        
        # Phase 3: Full stop and hold
        print('Phase 3: Full stop...')
        self.stop()
        time.sleep(1.0)
        
        print('=== MOVEMENT COMPLETE ===')


def main():
    rclpy.init()
    
    try:
        mover = FixedSpacecraftMover()
        time.sleep(1.0)
        
        mover.controlled_movement()
        
        time.sleep(2.0)
        
    except KeyboardInterrupt:
        print('Interrupted')
    finally:
        if 'mover' in locals():
            mover.stop()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
