#!/usr/bin/env python3

"""
IMU ENABLER FOR SPACECRAFT
==========================

This script helps you enable IMU publishing in your Isaac Sim spacecraft configuration.

The issue: Isaac Sim has IMU frames configured but no actual IMU sensor is created.
The solution: We need to add an IMU sensor to the spacecraft configuration.

STEPS TO ENABLE IMU:
1. Run this script to check current status
2. Follow the instructions to enable IMU in Isaac Sim
3. Use imu_pid_controller.py for IMU-based control

"""

import rclpy
import time
import subprocess
import sys

def check_imu_topics():
    """Check if IMU topics are being published"""
    print("🔍 Checking for IMU topics...")
    
    try:
        # Run ros2 topic list to check for IMU
        result = subprocess.run(['ros2', 'topic', 'list'], 
                              capture_output=True, text=True, timeout=10)
        
        if result.returncode == 0:
            topics = result.stdout.split('\n')
            imu_topics = [t for t in topics if 'imu' in t.lower()]
            
            print(f"\n📊 CURRENT ROS TOPICS:")
            for topic in topics:
                if topic.strip():
                    print(f"  - {topic}")
                    
            if imu_topics:
                print(f"\n✅ IMU TOPICS FOUND:")
                for topic in imu_topics:
                    print(f"  🎯 {topic}")
                return True
            else:
                print(f"\n❌ NO IMU TOPICS FOUND!")
                print(f"Expected: /srb/env0/imu_robot")
                return False
        else:
            print(f"❌ Error running ros2 topic list: {result.stderr}")
            return False
            
    except subprocess.TimeoutExpired:
        print("❌ Timeout waiting for ros2 topic list")
        return False
    except Exception as e:
        print(f"❌ Error: {e}")
        return False

def check_imu_data():
    """Try to listen for IMU data"""
    print("🔍 Checking for IMU data...")
    
    try:
        rclpy.init()
        
        from sensor_msgs.msg import Imu
        import rclpy
        from rclpy.node import Node
        
        class IMUChecker(Node):
            def __init__(self):
                super().__init__('imu_checker')
                self.imu_received = False
                
                self.subscription = self.create_subscription(
                    Imu,
                    '/srb/env0/imu_robot',
                    self.imu_callback,
                    10
                )
                
            def imu_callback(self, msg):
                if not self.imu_received:
                    print("✅ IMU DATA RECEIVED!")
                    print(f"📊 Linear Acceleration: [{msg.linear_acceleration.x:.6f}, {msg.linear_acceleration.y:.6f}, {msg.linear_acceleration.z:.6f}] m/s²")
                    print(f"📊 Angular Velocity: [{msg.angular_velocity.x:.6f}, {msg.angular_velocity.y:.6f}, {msg.angular_velocity.z:.6f}] rad/s")
                    self.imu_received = True
        
        checker = IMUChecker()
        
        print("⏳ Listening for IMU data for 5 seconds...")
        start_time = time.time()
        
        while time.time() - start_time < 5.0:
            rclpy.spin_once(checker, timeout_sec=0.1)
            if checker.imu_received:
                return True
                
        print("❌ No IMU data received in 5 seconds")
        return False
        
    except ImportError:
        print("❌ ROS2 not properly installed or configured")
        return False
    except Exception as e:
        print(f"❌ Error checking IMU data: {e}")
        return False
    finally:
        try:
            rclpy.shutdown()
        except:
            pass

def print_imu_instructions():
    """Print instructions for enabling IMU"""
    print("""
🚀 HOW TO ENABLE IMU IN ISAAC SIM:

The problem is that while your spacecraft has IMU frames defined, the actual IMU sensor
isn't being created and published. Here are the solutions:

📋 SOLUTION 1: Check Isaac Sim Configuration
1. In Isaac Sim, go to Window → Extensions
2. Search for "ROS2" and make sure it's enabled
3. Check if sensors are properly configured in your scene

📋 SOLUTION 2: Enable IMU in Spacecraft Configuration  
Your spacecraft config has IMU frames but no sensor. We need to:
1. Make sure the IMU sensor is attached to the spacecraft
2. Verify ROS interface is publishing sensor data

📋 SOLUTION 3: Verify Isaac Sim is Running
1. Make sure Isaac Sim is running with ROS interface
2. Check that your spacecraft environment is loaded
3. Verify ROS2_DOMAIN_ID matches between Isaac Sim and terminal

📋 SOLUTION 4: Manual IMU Creation
If automatic IMU doesn't work, we can create a physics-based IMU
that calculates acceleration from force/mass.

🎯 EXPECTED RESULT:
When working correctly, you should see:
  - Topic: /srb/env0/imu_robot (sensor_msgs/Imu)
  - Real acceleration data from spacecraft physics
  - IMU-based PID controller can integrate acceleration → velocity

🔧 TROUBLESHOOTING:
1. Check ROS_DOMAIN_ID: echo $ROS_DOMAIN_ID
2. Check Isaac Sim ROS settings
3. Try restarting Isaac Sim with ROS interface enabled
4. Verify spacecraft model has sensors configured

💡 ALTERNATIVE:
If IMU can't be enabled, we can create a hybrid approach:
- Monitor thrust commands to estimate acceleration  
- Use physics-based integration for velocity
- Still more accurate than our current simulation
""")

def main():
    print("🎯 IMU ENABLER FOR SPACECRAFT")
    print("="*50)
    
    # Check if topics exist
    imu_topics_exist = check_imu_topics()
    
    if imu_topics_exist:
        # Try to get actual data
        imu_data_exists = check_imu_data()
        
        if imu_data_exists:
            print("\n🎉 SUCCESS! IMU is working correctly!")
            print("✅ You can now use imu_pid_controller.py for IMU-based control")
            print("\nTo start IMU-based PID control:")
            print("  cd /home/bfarhat/SURF/space_robotics_bench")
            print("  python3 imu_pid_controller.py")
        else:
            print("\n⚠️  IMU topic exists but no data is being published")
            print("This usually means the IMU sensor isn't properly configured")
            print_imu_instructions()
    else:
        print("\n❌ IMU topics don't exist - IMU not enabled in Isaac Sim")
        print_imu_instructions()

if __name__ == '__main__':
    main()

