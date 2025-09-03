#!/usr/bin/env python3

"""
SPACECRAFT IMU SYSTEM LAUNCHER
==============================

This script helps you start the complete IMU system for spacecraft control.
It provides options to test IMU data and use it for precise control.

WHAT THIS SCRIPT DOES:
✅ Checks for existing IMU topics from Isaac Sim
✅ Starts IMU node if needed (physics-based or simulated)
✅ Tests IMU data reception
✅ Launches IMU-based PID controller
✅ Provides integrated system management

USAGE:
    cd /home/bfarhat/SURF/space_robotics_bench
    python3 start_imu_system.py
"""

import subprocess
import time
import sys
import threading
import signal

def check_ros_topics():
    """Check what ROS topics are currently available"""
    print("🔍 Checking current ROS topics...")
    
    try:
        # Try to get topic list
        result = subprocess.run(['ros2', 'topic', 'list'], 
                              capture_output=True, text=True, timeout=5)
        
        if result.returncode == 0:
            topics = [t.strip() for t in result.stdout.split('\n') if t.strip()]
            
            print(f"\n📊 CURRENT ROS TOPICS ({len(topics)} found):")
            for topic in topics:
                if 'imu' in topic.lower():
                    print(f"  🎯 {topic} (IMU-related)")
                elif 'thrust' in topic.lower():
                    print(f"  🚀 {topic} (Thrust-related)")
                elif topic.startswith('/srb/'):
                    print(f"  🛰️ {topic} (SRB-related)")
                else:
                    print(f"  📡 {topic}")
            
            # Check for specific IMU topics
            imu_topics = [t for t in topics if 'imu' in t.lower()]
            thrust_topics = [t for t in topics if 'thrust' in t.lower()]
            
            return {
                'all_topics': topics,
                'imu_topics': imu_topics,
                'thrust_topics': thrust_topics,
                'has_isaac_imu': '/srb/env0/imu_robot' in topics,
                'has_thrust': '/srb/env0/robot/thrust' in topics
            }
        else:
            print(f"❌ Error getting topics: {result.stderr}")
            return None
            
    except subprocess.TimeoutExpired:
        print("⚠️ Timeout getting ROS topics")
        return None
    except Exception as e:
        print(f"❌ Error: {e}")
        return None

def test_imu_data(topic_name, timeout=5):
    """Test if IMU data is being published on a topic"""
    print(f"🔍 Testing IMU data on {topic_name} for {timeout} seconds...")
    
    try:
        # Use ros2 topic echo to check for data
        process = subprocess.Popen(
            ['ros2', 'topic', 'echo', topic_name, '--once'],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )
        
        stdout, stderr = process.communicate(timeout=timeout)
        
        if process.returncode == 0 and stdout.strip():
            print(f"✅ IMU data received on {topic_name}!")
            print("📊 Sample data preview:")
            lines = stdout.split('\n')[:10]  # Show first 10 lines
            for line in lines:
                if line.strip():
                    print(f"    {line}")
            return True
        else:
            print(f"❌ No IMU data on {topic_name}")
            if stderr:
                print(f"   Error: {stderr}")
            return False
            
    except subprocess.TimeoutExpired:
        process.kill()
        print(f"⚠️ Timeout waiting for data on {topic_name}")
        return False
    except Exception as e:
        print(f"❌ Error testing {topic_name}: {e}")
        return False

def start_imu_node(mode='physics'):
    """Start the spacecraft IMU node"""
    print(f"🚀 Starting IMU node in {mode} mode...")
    
    if mode == 'physics':
        choice = '2'  # Physics-based IMU
        print("📊 Physics-based IMU: Calculates acceleration from thrust commands")
    else:
        choice = '1'  # Simulated IMU
        print("📊 Simulated IMU: Generates realistic test data")
    
    try:
        # Start IMU node as subprocess
        process = subprocess.Popen(
            ['python3', 'spacecraft_imu_node.py'],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            cwd='/home/bfarhat/SURF/space_robotics_bench'
        )
        
        # Send choice to the process
        process.stdin.write(f"{choice}\n")
        process.stdin.flush()
        
        # Give it time to start
        time.sleep(2)
        
        if process.poll() is None:  # Still running
            print("✅ IMU node started successfully!")
            return process
        else:
            stdout, _ = process.communicate()
            print(f"❌ IMU node failed to start:")
            print(stdout)
            return None
            
    except Exception as e:
        print(f"❌ Error starting IMU node: {e}")
        return None

def start_imu_pid_controller():
    """Start the IMU-based PID controller"""
    print("🎯 Starting IMU-based PID controller...")
    
    try:
        process = subprocess.Popen(
            ['python3', 'imu_pid_controller.py'],
            cwd='/home/bfarhat/SURF/space_robotics_bench'
        )
        
        return process
        
    except Exception as e:
        print(f"❌ Error starting IMU PID controller: {e}")
        return None

def main():
    print("🛰️ SPACECRAFT IMU SYSTEM LAUNCHER")
    print("="*50)
    
    # Check current ROS environment
    topic_info = check_ros_topics()
    
    if topic_info is None:
        print("❌ Cannot check ROS topics. Make sure ROS2 is running.")
        return
    
    print(f"\n🎯 SYSTEM STATUS:")
    print(f"  Isaac Sim IMU: {'✅ Available' if topic_info['has_isaac_imu'] else '❌ Not found'}")
    print(f"  Thrust topics: {'✅ Available' if topic_info['has_thrust'] else '❌ Not found'}")
    print(f"  Total topics: {len(topic_info['all_topics'])}")
    
    # Determine best course of action
    if topic_info['has_isaac_imu']:
        print(f"\n🎉 EXCELLENT! Isaac Sim is publishing real IMU data!")
        print(f"📡 Topic: /srb/env0/imu_robot")
        
        # Test the real IMU data
        if test_imu_data('/srb/env0/imu_robot'):
            print(f"\n🎯 Real IMU data confirmed! You can directly use:")
            print(f"   python3 imu_pid_controller.py")
            
            choice = input(f"\nStart IMU-based PID controller now? (y/n): ").strip().lower()
            if choice == 'y':
                controller_process = start_imu_pid_controller()
                if controller_process:
                    try:
                        controller_process.wait()
                    except KeyboardInterrupt:
                        print(f"\n🛑 Stopping IMU PID controller...")
                        controller_process.terminate()
            return
        else:
            print(f"⚠️ IMU topic exists but no data. Check Isaac Sim configuration.")
    
    elif topic_info['has_thrust']:
        print(f"\n🔄 Isaac Sim thrust topics found, but no IMU.")
        print(f"💡 Solution: Start physics-based IMU node")
        
        choice = input(f"\nStart physics-based IMU node? (y/n): ").strip().lower()
        if choice == 'y':
            imu_process = start_imu_node('physics')
            if imu_process:
                print(f"\n⏳ Waiting for IMU node to initialize...")
                time.sleep(3)
                
                # Test the physics-based IMU
                if test_imu_data('/srb/env0/imu_robot', timeout=3):
                    print(f"\n🎯 Physics-based IMU working! Starting PID controller...")
                    
                    controller_process = start_imu_pid_controller()
                    if controller_process:
                        try:
                            # Run both processes
                            print(f"✅ Both IMU node and PID controller running!")
                            print(f"🎯 Use the PID controller menu to test spacecraft control")
                            print(f"🛑 Press Ctrl+C to stop both processes")
                            
                            controller_process.wait()
                        except KeyboardInterrupt:
                            print(f"\n🛑 Stopping all processes...")
                            controller_process.terminate()
                            imu_process.terminate()
                else:
                    print(f"❌ Physics-based IMU not working correctly")
                    imu_process.terminate()
    
    else:
        print(f"\n⚠️ No Isaac Sim topics found.")
        print(f"💡 Solutions:")
        print(f"1. Make sure Isaac Sim is running with ROS interface")
        print(f"2. Load spacecraft environment in Isaac Sim")
        print(f"3. Start simulated IMU node for testing")
        
        choice = input(f"\nStart simulated IMU node for testing? (y/n): ").strip().lower()
        if choice == 'y':
            imu_process = start_imu_node('simulated')
            if imu_process:
                time.sleep(3)
                if test_imu_data('/srb/env0/imu_robot', timeout=3):
                    print(f"\n✅ Simulated IMU working for testing!")
                    
                    choice = input(f"Start PID controller with simulated data? (y/n): ").strip().lower()
                    if choice == 'y':
                        controller_process = start_imu_pid_controller()
                        if controller_process:
                            try:
                                controller_process.wait()
                            except KeyboardInterrupt:
                                print(f"\n🛑 Stopping...")
                                controller_process.terminate()
                                imu_process.terminate()
                else:
                    print(f"❌ Simulated IMU failed")
                    imu_process.terminate()
    
    print(f"\n📋 NEXT STEPS:")
    print(f"1. Make sure Isaac Sim is running with spacecraft loaded")
    print(f"2. Enable IMU sensor using spacecraft_with_imu_config.py")
    print(f"3. Use start_imu_system.py to test the complete system")
    print(f"4. Run imu_pid_controller.py for ultra-precise control!")

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print(f"\n🛑 Interrupted by user")
    except Exception as e:
        print(f"❌ Error: {e}")
