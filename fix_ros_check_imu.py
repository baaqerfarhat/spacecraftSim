#!/usr/bin/env python3

"""
FIX ROS AND CHECK IMU
====================

Fixes the ROS2 netifaces issue and checks IMU without using ros2 CLI.
This bypasses the ros2 command line tool that's having dependency issues.
"""

import sys
import time

def check_ros_topics_directly():
    """Check ROS topics using Python ROS client instead of CLI"""
    print("🔍 CHECKING ROS TOPICS (bypassing CLI issues)...")
    print("="*50)
    
    try:
        import rclpy
        from rclpy.node import Node
        
        # Initialize ROS
        rclpy.init()
        
        # Create a temporary node to discover topics
        node = Node('topic_checker')
        
        print("✅ ROS2 Python client working!")
        
        # Get topic names and types
        topic_names_and_types = node.get_topic_names_and_types()
        
        print(f"📊 Total topics found: {len(topic_names_and_types)}")
        
        # Analyze topics
        all_topics = [name for name, types in topic_names_and_types]
        srb_topics = [t for t in all_topics if '/srb/' in t]
        imu_topics = [t for t in all_topics if 'imu' in t.lower()]
        thrust_topics = [t for t in all_topics if 'thrust' in t]
        
        print(f"\n🛰️  SRB topics ({len(srb_topics)}):")
        for topic in srb_topics:
            topic_type = [types[0] for name, types in topic_names_and_types if name == topic][0]
            print(f"   {topic} ({topic_type})")
        
        print(f"\n🎯 IMU topics ({len(imu_topics)}):")
        for topic in imu_topics:
            topic_type = [types[0] for name, types in topic_names_and_types if name == topic][0]
            print(f"   {topic} ({topic_type})")
            
        print(f"\n🚀 Thrust topics ({len(thrust_topics)}):")
        for topic in thrust_topics:
            topic_type = [types[0] for name, types in topic_names_and_types if name == topic][0]
            print(f"   {topic} ({topic_type})")
        
        # Cleanup
        node.destroy_node()
        rclpy.shutdown()
        
        return len(imu_topics) > 0, len(srb_topics) > 0, all_topics
        
    except ImportError:
        print("❌ ROS2 Python client not available")
        return False, False, []
    except Exception as e:
        print(f"❌ Error checking topics: {e}")
        return False, False, []

def test_imu_data_directly():
    """Test IMU data using Python ROS client"""
    print("\n🔍 TESTING IMU DATA (bypassing CLI)...")
    print("="*40)
    
    try:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import Imu
        
        rclpy.init()
        
        class IMUTester(Node):
            def __init__(self):
                super().__init__('imu_tester')
                self.imu_received = False
                self.imu_data = None
                
                # Try to subscribe to IMU topic
                self.subscription = self.create_subscription(
                    Imu,
                    '/srb/env0/imu_robot',
                    self.imu_callback,
                    10
                )
                
            def imu_callback(self, msg):
                if not self.imu_received:
                    self.imu_received = True
                    self.imu_data = msg
                    print("✅ IMU DATA RECEIVED!")
                    print(f"📊 Linear acceleration: [{msg.linear_acceleration.x:.6f}, {msg.linear_acceleration.y:.6f}, {msg.linear_acceleration.z:.6f}] m/s²")
                    print(f"📊 Angular velocity: [{msg.angular_velocity.x:.6f}, {msg.angular_velocity.y:.6f}, {msg.angular_velocity.z:.6f}] rad/s")
        
        node = IMUTester()
        
        print("⏳ Listening for IMU data for 5 seconds...")
        start_time = time.time()
        
        while time.time() - start_time < 5.0 and not node.imu_received:
            rclpy.spin_once(node, timeout_sec=0.1)
        
        success = node.imu_received
        
        if not success:
            print("❌ No IMU data received in 5 seconds")
        
        node.destroy_node()
        rclpy.shutdown()
        
        return success
        
    except Exception as e:
        print(f"❌ Error testing IMU: {e}")
        return False

def check_isaac_sim_container_setup():
    """Check if running in container with proper Isaac Sim setup"""
    print("\n🐳 CHECKING CONTAINER SETUP...")
    print("="*40)
    
    try:
        import os
        
        # Check if in container
        in_container = os.path.exists('/.dockerenv') or 'container' in os.environ.get('HOSTNAME', '')
        print(f"Running in container: {in_container}")
        
        # Check Isaac Sim environment
        isaac_path = os.environ.get('ISAAC_SIM_PATH', '/isaac-sim')
        print(f"Isaac Sim path: {isaac_path}")
        print(f"Isaac Sim exists: {os.path.exists(isaac_path)}")
        
        # Check CUDA/GPU setup in container
        cuda_visible = os.environ.get('CUDA_VISIBLE_DEVICES', 'Not set')
        print(f"CUDA_VISIBLE_DEVICES: {cuda_visible}")
        
        # Check if running with --gpus flag
        nvidia_visible = os.environ.get('NVIDIA_VISIBLE_DEVICES', 'Not set')
        print(f"NVIDIA_VISIBLE_DEVICES: {nvidia_visible}")
        
        return in_container
        
    except Exception as e:
        print(f"❌ Error checking container: {e}")
        return False

def provide_container_fixes():
    """Provide fixes for container setup"""
    print("\n🔧 CONTAINER FIXES")
    print("="*40)
    
    print("📋 Based on the errors, your Isaac Sim container needs:")
    print()
    print("1. 🎮 GPU ACCESS:")
    print("   Make sure container started with:")
    print("   docker run --gpus all ...")
    print("   or")
    print("   docker run --runtime=nvidia ...")
    print()
    print("2. 📦 MISSING ROS DEPENDENCY:")
    print("   pip install netifaces")
    print("   (This will fix the ros2 topic list error)")
    print()
    print("3. 🖥️  DISPLAY/X11 (if using GUI):")
    print("   docker run -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix ...")
    print()
    print("4. 🚀 TRY HEADLESS MODE:")
    print("   srb agent ros -e orbital_evasion env.robot=lab_sc --headless")
    print("   (This avoids GPU rendering issues)")

def main():
    print("🛰️  FIX ROS AND CHECK IMU")
    print("="*50)
    print("Fixing ROS2 CLI issues and checking IMU directly...")
    print("="*50)
    
    # Check container setup
    in_container = check_isaac_sim_container_setup()
    
    # Check ROS topics directly (bypass CLI issues)
    has_imu, has_srb, all_topics = check_ros_topics_directly()
    
    # Test IMU if available
    imu_working = False
    if has_imu:
        imu_working = test_imu_data_directly()
    
    # Provide diagnosis
    print(f"\n🎯 DIAGNOSIS")
    print("="*40)
    
    if imu_working:
        print("🎉 SUCCESS! IMU is working!")
        print("✅ Use: python3 imu_pid_controller.py")
        
    elif has_srb and not has_imu:
        print("⚠️  Isaac Sim running but NO IMU sensor")
        print("📋 Need to enable IMU in spacecraft configuration")
        print("💡 Try: python3 physics_imu_node.py")
        print("   (This creates IMU from thrust commands)")
        
    elif not has_srb:
        print("❌ Isaac Sim not properly connected to ROS")
        if in_container:
            provide_container_fixes()
        else:
            print("🔄 Try restarting Isaac Sim with proper ROS interface")
    
    print(f"\n📋 SUMMARY:")
    print(f"Container detected: {'✅' if in_container else '❌'}")
    print(f"ROS topics available: {'✅' if has_srb else '❌'}")
    print(f"IMU topic exists: {'✅' if has_imu else '❌'}")
    print(f"IMU data working: {'✅' if imu_working else '❌'}")
    
    # Quick fix for ROS CLI
    print(f"\n🔧 QUICK FIX FOR ROS CLI:")
    print(f"pip install netifaces")
    print(f"(This will fix the 'ros2 topic list' command)")

if __name__ == '__main__':
    main()
