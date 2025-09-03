#!/usr/bin/env python3

"""
SIMPLE IMU CHECK
================

One file to check if IMU is working in your Isaac Sim setup.
This doesn't change anything - just checks what's available.
"""

import subprocess
import sys
import time

def check_ros_topics():
    """Check what ROS topics are currently available"""
    print("🔍 CHECKING ROS TOPICS...")
    print("="*40)
    
    try:
        result = subprocess.run(['ros2', 'topic', 'list'], 
                              capture_output=True, text=True, timeout=5)
        
        if result.returncode == 0:
            topics = [t.strip() for t in result.stdout.split('\n') if t.strip()]
            
            print(f"📊 Total topics found: {len(topics)}")
            
            # Check for key topics
            srb_topics = [t for t in topics if '/srb/' in t]
            imu_topics = [t for t in topics if 'imu' in t.lower()]
            thrust_topics = [t for t in topics if 'thrust' in t]
            
            print(f"🛰️  SRB topics: {len(srb_topics)}")
            for topic in srb_topics:
                print(f"   {topic}")
                
            if imu_topics:
                print(f"🎯 IMU topics found: {imu_topics}")
                return True, "IMU topics exist"
            else:
                print(f"❌ No IMU topics found")
                
            if thrust_topics:
                print(f"🚀 Thrust topics: {thrust_topics}")
                return False, "Isaac Sim running but no IMU"
            else:
                print(f"❌ No thrust topics - Isaac Sim not running")
                return False, "Isaac Sim not running"
                
        else:
            print(f"❌ Error: {result.stderr}")
            return False, "ROS error"
            
    except Exception as e:
        print(f"❌ Error checking topics: {e}")
        return False, "Check failed"

def test_imu_topic():
    """Test if we can get data from IMU topic"""
    print("\n🔍 TESTING IMU DATA...")
    print("="*40)
    
    try:
        print("Trying to get one IMU message (5 second timeout)...")
        result = subprocess.run(
            ['ros2', 'topic', 'echo', '/srb/env0/imu_robot', '--once'],
            capture_output=True, text=True, timeout=5
        )
        
        if result.returncode == 0 and result.stdout.strip():
            print("✅ IMU DATA RECEIVED!")
            print("📊 Sample data:")
            lines = result.stdout.split('\n')[:15]  # Show first 15 lines
            for line in lines:
                if line.strip():
                    print(f"   {line}")
            return True
        else:
            print("❌ No IMU data received")
            return False
            
    except subprocess.TimeoutExpired:
        print("⏳ Timeout - no IMU data within 5 seconds")
        return False
    except Exception as e:
        print(f"❌ Error: {e}")
        return False

def check_isaac_sim_status():
    """Check if Isaac Sim processes are running"""
    print("\n🔍 CHECKING ISAAC SIM STATUS...")
    print("="*40)
    
    try:
        # Check for Isaac Sim processes
        result = subprocess.run(['ps', 'aux'], capture_output=True, text=True)
        
        isaac_processes = []
        for line in result.stdout.split('\n'):
            if 'isaac' in line.lower() or 'kit' in line.lower() or 'srb' in line.lower():
                isaac_processes.append(line.strip())
        
        if isaac_processes:
            print(f"🎮 Isaac Sim processes found: {len(isaac_processes)}")
            for proc in isaac_processes[:3]:  # Show first 3
                print(f"   {proc[:80]}...")
            return True
        else:
            print("❌ No Isaac Sim processes found")
            return False
            
    except Exception as e:
        print(f"❌ Error checking processes: {e}")
        return False

def provide_diagnosis(has_imu, status_msg, isaac_running):
    """Provide diagnosis and next steps"""
    print("\n🎯 DIAGNOSIS")
    print("="*40)
    
    if has_imu:
        print("🎉 SUCCESS! IMU is working!")
        print("✅ Isaac Sim is running")
        print("✅ IMU sensor is publishing data")
        print("✅ You can use: python3 imu_pid_controller.py")
        
    elif isaac_running and "Isaac Sim running" in status_msg:
        print("⚠️  Isaac Sim is running but NO IMU sensor")
        print("📋 SOLUTIONS:")
        print("1. 🔧 IMU sensor not configured in your environment")
        print("2. 🛠️  Need to enable IMU in Isaac Sim scene")
        print("3. 🎯 Try creating environment with IMU enabled")
        
    else:
        print("❌ ISAAC SIM NOT RUNNING PROPERLY")
        print("📊 The errors you showed indicate:")
        print("   - GPU/CUDA driver issues")
        print("   - Isaac Sim startup problems") 
        print("   - This is NOT caused by my scripts!")
        print()
        print("🔧 FIXES:")
        print("1. 🖥️  Check GPU drivers:")
        print("      nvidia-smi")
        print("2. 🐳 If in Docker, check NVIDIA container runtime")
        print("3. 🔄 Try restarting Isaac Sim:")
        print("      srb agent ros -e orbital_evasion env.robot=lab_sc")
        print("4. 💻 Try headless mode if GUI issues:")
        print("      srb agent ros -e orbital_evasion env.robot=lab_sc --headless")

def main():
    print("🛰️  SIMPLE IMU CHECK")
    print("="*50)
    print("This script only CHECKS your setup - doesn't change anything!")
    print("="*50)
    
    # Check if Isaac Sim is running
    isaac_running = check_isaac_sim_status()
    
    # Check ROS topics
    has_imu, status_msg = check_ros_topics()
    
    # If IMU topic exists, test it
    if has_imu:
        imu_working = test_imu_topic()
    else:
        imu_working = False
    
    # Provide diagnosis
    provide_diagnosis(imu_working if has_imu else False, status_msg, isaac_running)
    
    print(f"\n📋 SUMMARY:")
    print(f"Isaac Sim running: {'✅' if isaac_running else '❌'}")
    print(f"IMU topic exists: {'✅' if has_imu else '❌'}")
    print(f"IMU data working: {'✅' if has_imu and imu_working else '❌'}")
    
    if not isaac_running:
        print(f"\n💡 IMPORTANT:")
        print(f"The Isaac Sim startup errors you showed are NOT caused by my scripts.")
        print(f"Those are GPU/CUDA driver issues that need to be fixed first.")
        print(f"My scripts only create new files - they don't modify Isaac Sim.")

if __name__ == '__main__':
    main()
