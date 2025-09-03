import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import time


class GentleSpacecraftMover(Node):
    def __init__(self):
        super().__init__('gentle_spacecraft_mover')
        
        self.thruster_pub = self.create_publisher(
            Float32MultiArray, 
            '/srb/env0/robot/thrust', 
            10
        )
        
        # Use forward/back thrusters for predictable movement
        self.FORWARD_THRUSTERS = [4, 5]  # +Y direction
        self.BACK_THRUSTERS = [6, 7]     # -Y direction
        
        self.get_logger().info('Gentle Spacecraft Mover Started - VERY LOW POWER')
        
    def send_command(self, thrust_values):
        msg = Float32MultiArray()
        msg.data = thrust_values
        self.thruster_pub.publish(msg)
        self.get_logger().info(f'Sent: {thrust_values}')
        
    def stop(self):
        self.send_command([0.0] * 8)
        
    def move_forward(self, power=0.05):  # VERY LOW power - only 5%
        cmd = [0.0] * 8
        for i in self.FORWARD_THRUSTERS:
            cmd[i] = power
        self.send_command(cmd)
        
    def move_back(self, power=0.05):  # VERY LOW power - only 5%
        cmd = [0.0] * 8
        for i in self.BACK_THRUSTERS:
            cmd[i] = power
        self.send_command(cmd)
        
    def gentle_movement(self):
        print('=== GENTLE MOVEMENT TEST ===')
        
        # Phase 1: Very gentle forward push
        print('Phase 1: Gentle FORWARD push for 1 second at 5% power...')
        self.move_forward(0.05)
        time.sleep(1.0)
        
        # Phase 2: Longer gentle braking
        print('Phase 2: Gentle braking for 2 seconds...')
        self.move_back(0.05)
        time.sleep(2.0)
        
        # Phase 3: Stop
        print('Phase 3: Full stop')
        self.stop()
        time.sleep(1.0)
        
        print('=== GENTLE MOVEMENT COMPLETE ===')


def main():
    rclpy.init()
    
    try:
        mover = GentleSpacecraftMover()
        time.sleep(1.0)
        
        mover.gentle_movement()
        
        time.sleep(1.0)
        
    except KeyboardInterrupt:
        print('Interrupted')
    finally:
        if 'mover' in locals():
            mover.stop()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
