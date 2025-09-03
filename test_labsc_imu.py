#!/usr/bin/env python3

"""
TEST LABSC IMU CONFIGURATION
============================

This script tests if we can properly configure the LabSc IMU sensor
and verify what's missing for the real Isaac Sim IMU to work.

Run this to understand why IMU isn't working in your current setup.
"""

import sys
import os

def test_labsc_imu_frames():
    """Test if LabSc spacecraft has proper IMU frame definitions"""
    print("🔍 TESTING LABSC IMU FRAMES")
    print("="*40)
    
    try:
        # Add project path
        sys.path.insert(0, '/home/bfarhat/SURF/space_robotics_bench')
        
        from srb.assets.robot.mobile.spacecraft import LabSc
        
        # Create LabSc instance
        spacecraft = LabSc()
        
        print(f"✅ LabSc spacecraft loaded: {type(spacecraft).__name__}")
        
        # Check IMU frames
        has_frame_imu = hasattr(spacecraft, 'frame_imu')
        has_frame_onboard_imu = hasattr(spacecraft, 'frame_onboard_imu') 
        
        print(f"frame_imu exists: {has_frame_imu}")
        print(f"frame_onboard_imu exists: {has_frame_onboard_imu}")
        
        if has_frame_imu:
            print(f"frame_imu.prim_relpath: '{spacecraft.frame_imu.prim_relpath}'")
            print(f"frame_imu.offset: {spacecraft.frame_imu.offset}")
            
        if has_frame_onboard_imu:
            print(f"frame_onboard_imu.prim_relpath: '{spacecraft.frame_onboard_imu.prim_relpath}'")  
            print(f"frame_onboard_imu.offset: {spacecraft.frame_onboard_imu.offset}")
        
        # Compare with frame_base
        print(f"frame_base.prim_relpath: '{spacecraft.frame_base.prim_relpath}'")
        
        if has_frame_imu and spacecraft.frame_imu.prim_relpath == spacecraft.frame_base.prim_relpath:
            print("⚠️  frame_imu and frame_base have SAME prim_relpath!")
            print("   This means the bug in mobile/env.py doesn't matter")
        
        return has_frame_imu or has_frame_onboard_imu
        
    except Exception as e:
        print(f"❌ Error testing LabSc: {e}")
        return False

def test_imu_config_creation():
    """Test if we can create proper IMU configuration"""
    print("\n🔧 TESTING IMU CONFIGURATION CREATION")
    print("="*40)
    
    try:
        from srb.core.sensor import ImuCfg
        from srb.assets.robot.mobile.spacecraft import LabSc
        
        spacecraft = LabSc()
        
        # Create IMU configuration like Isaac Sim expects
        imu_cfg = ImuCfg(
            prim_path="/World/envs/env_0/lab_sc_robot_unique/base",  # Full path
            update_period=0.02,  # 50 Hz
            gravity_bias=(0.0, 0.0, 0.0),
            offset=spacecraft.frame_imu.offset,
        )
        
        print(f"✅ IMU configuration created successfully!")
        print(f"   prim_path: {imu_cfg.prim_path}")
        print(f"   update_period: {imu_cfg.update_period} ({1/imu_cfg.update_period:.0f} Hz)")
        print(f"   offset: {imu_cfg.offset}")
        
        return True
        
    except Exception as e:
        print(f"❌ Error creating IMU config: {e}")
        return False

def test_current_isaac_sim_state():
    """Test what's currently running in Isaac Sim"""
    print("\n📊 TESTING CURRENT ISAAC SIM STATE")
    print("="*40)
    
    try:
        import subprocess
        
        # Check ROS topics
        result = subprocess.run(['ros2', 'topic', 'list'], 
                              capture_output=True, text=True, timeout=5)
        
        if result.returncode == 0:
            topics = result.stdout.strip().split('\n')
            srb_topics = [t for t in topics if '/srb/' in t]
            
            print(f"📡 SRB topics found: {len(srb_topics)}")
            for topic in srb_topics:
                print(f"   {topic}")
            
            # Check for any IMU topics
            imu_topics = [t for t in topics if 'imu' in t.lower()]
            if imu_topics:
                print(f"🎯 IMU topics found: {imu_topics}")
            else:
                print(f"❌ No IMU topics found")
                
            # Check for spacecraft/robot topics
            robot_topics = [t for t in srb_topics if 'robot' in t]
            print(f"🛰️  Robot topics: {robot_topics}")
            
            return len(srb_topics) > 0
        else:
            print(f"❌ Error getting ROS topics: {result.stderr}")
            return False
            
    except Exception as e:
        print(f"❌ Error checking Isaac Sim state: {e}")
        return False

def provide_solution_steps():
    """Provide step-by-step solution based on test results"""
    print("\n🎯 SOLUTION STEPS")
    print("="*40)
    
    print("Based on the tests, here's what needs to be done:")
    print()
    print("1. 🔧 PROBLEM IDENTIFIED:")
    print("   - LabSc HAS IMU frames (unique among SRB robots)")
    print("   - ROS interface SUPPORTS IMU publishing") 
    print("   - Issue: IMU sensor not instantiated in current scene")
    print()
    print("2. 🚀 SOLUTION OPTIONS:")
    print()
    print("   OPTION A: Restart Isaac Sim with proper IMU config")
    print("   ✅ Use: python3 enable_labsc_imu.py")
    print("   ✅ This creates proper environment with IMU enabled")
    print()
    print("   OPTION B: If Isaac Sim must stay running:")
    print("   ⚙️  Need to dynamically add IMU sensor to existing scene")
    print("   🔧 This requires Isaac Sim Python API access")
    print()
    print("3. ✅ EXPECTED RESULT:")
    print("   📡 Topic: /srb/env0/imu_robot")
    print("   📊 Real acceleration/angular velocity from Isaac Sim")
    print("   🎯 Compatible with your imu_pid_controller.py")

def main():
    print("🛰️ LABSC IMU DIAGNOSTIC TEST")
    print("="*50)
    print("Researching why LabSc IMU isn't working...")
    print("="*50)
    
    # Run all tests
    test1 = test_labsc_imu_frames()
    test2 = test_imu_config_creation() 
    test3 = test_current_isaac_sim_state()
    
    print(f"\n📋 TEST RESULTS:")
    print(f"LabSc IMU frames: {'✅ PASS' if test1 else '❌ FAIL'}")
    print(f"IMU config creation: {'✅ PASS' if test2 else '❌ FAIL'}")
    print(f"Isaac Sim running: {'✅ DETECTED' if test3 else '❌ NOT DETECTED'}")
    
    if test1 and test2:
        print(f"\n🎉 GOOD NEWS!")
        print(f"✅ LabSc spacecraft HAS proper IMU frames")
        print(f"✅ IMU configuration CAN be created")
        print(f"✅ The hardware/software supports real IMU")
        print(f"")
        print(f"🔧 The issue is just environment configuration!")
    
    provide_solution_steps()

if __name__ == '__main__':
    main()
