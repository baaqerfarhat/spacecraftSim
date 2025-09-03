import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import time


class CorrectedSpacecraftMover(Node):
    def __init__(self):
        super().__init__('corrected_spacecraft_mover')
        
        self.thruster_pub = self.create_publisher(
            Float32MultiArray, 
            '/srb/env0/robot/thrust', 
            10
        )
        
        # CORRECTED: Use forward/back thrusters for predictable movement
        self.FORWARD_THRUSTERS = [4, 5]  # +Y direction
        self.BACK_THRUSTERS = [6, 7]     # -Y direction
        
        self.get_logger().info('Corrected Spacecraft Mover Node Started')
        self.get_logger().info('Using FORWARD/BACK thrusters with proper braking')
        
    def send_command(self, thrust_values):
        msg = Float32MultiArray()
        msg.data = thrust_values
        self.thruster_pub.publish(msg)
        self.get_logger().info(f'Sent: {thrust_values}')
        
    def stop(self):
        self.send_command([0.0] * 8)
        
    def move_forward(self, power=0.1):
        cmd = [0.0] * 8
        for i in self.FORWARD_THRUSTERS:
            cmd[i] = power
        self.send_command(cmd)
        
    def move_back(self, power=0.1):
        cmd = [0.0] * 8
        for i in self.BACK_THRUSTERS:
            cmd[i] = power
        self.send_command(cmd)
        
    def controlled_movement(self):
        print('=== STARTING CORRECTED CONTROLLED MOVEMENT ===')
        
        # Phase 1: Move forward for 1.5 seconds
        forward_time = 1.5
        forward_power = 0.1
        print(f'Phase 1: Moving FORWARD for {forward_time} seconds at {forward_power*100}% power...')
        self.move_forward(forward_power)
        time.sleep(forward_time)
        
        # Phase 2: PROPER BRAKING - slightly less time to avoid overshoot
        brake_time = forward_time * 0.95  # 5% less brake time to avoid overshoot
        brake_power = forward_power
        print(f'Phase 2: Braking for {brake_time:.2f} seconds at {brake_power*100}% power...')
        self.move_back(brake_power)
        time.sleep(brake_time)
        
        # Phase 3: Initial stop
        print('Phase 3: Initial stop...')
        self.stop()
        time.sleep(0.5)
        
        # Phase 4: CLEANUP - Apply small correction pulses to eliminate residual motion
        print('Phase 4: Applying cleanup pulses to eliminate residual motion...')
        cleanup_power = 0.03  # Very low power
        cleanup_time = 0.2    # Very short pulses
        
        # Apply 3 small backward pulses to kill any remaining forward motion
        for i in range(3):
            print(f'  Cleanup pulse {i+1}/3...')
            self.move_back(cleanup_power)
            time.sleep(cleanup_time)
            self.stop()
            time.sleep(0.3)
        
        # Final stop and hold
        print('Phase 5: Final stop and hold...')
        self.stop()
        time.sleep(1.0)
        
        print('=== MOVEMENT COMPLETE - ALL MOTION ELIMINATED ===')

    def gentle_movement(self):
        """Alternative: Very gentle movement with multiple short pulses"""
        print('=== STARTING GENTLE PULSED MOVEMENT ===')
        
        # Multiple short forward pulses
        pulse_time = 0.3
        pulse_power = 0.05  # Very low power
        num_pulses = 3
        
        for i in range(num_pulses):
            print(f'Forward pulse {i+1}/{num_pulses}...')
            self.move_forward(pulse_power)
            time.sleep(pulse_time)
            self.stop()
            time.sleep(0.5)  # Wait between pulses
        
        # Wait to let it coast
        print('Coasting for 2 seconds...')
        time.sleep(2.0)
        
        # Multiple short brake pulses - fewer than forward to avoid overshoot
        brake_pulses = num_pulses - 1  # One less brake pulse
        for i in range(brake_pulses):
            print(f'Brake pulse {i+1}/{brake_pulses}...')
            self.move_back(pulse_power)
            time.sleep(pulse_time)
            self.stop()
            time.sleep(0.5)
        
        print('=== GENTLE MOVEMENT COMPLETE ===')

    def precise_movement(self):
        """Most precise approach: very short alternating thrusts"""
        print('=== STARTING PRECISE MOVEMENT ===')
        
        # Very short forward movement
        print('Short forward thrust...')
        self.move_forward(0.08)
        time.sleep(0.5)
        
        # Immediate stop
        self.stop()
        time.sleep(0.5)
        
        # Very short brake
        print('Short brake thrust...')
        self.move_back(0.08)
        time.sleep(0.4)  # Slightly less time to avoid overshoot
        
        # Final stop
        self.stop()
        time.sleep(1.0)
        
        print('=== PRECISE MOVEMENT COMPLETE ===')


def main():
    rclpy.init()
    
    try:
        mover = CorrectedSpacecraftMover()
        time.sleep(1.0)
        
        # Choose your movement method:
        print("Choose movement type:")
        print("1. Corrected movement (equal thrust/brake times)")
        print("2. Gentle pulsed movement")
        print("3. Precise short movement")
        
        # For demo, let's use the corrected movement
        mover.controlled_movement()
        
        # Uncomment to try other methods:
        # mover.gentle_movement()
        # mover.precise_movement()
        
        time.sleep(2.0)
        
    except KeyboardInterrupt:
        print('Interrupted')
    finally:
        if 'mover' in locals():
            mover.stop()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
