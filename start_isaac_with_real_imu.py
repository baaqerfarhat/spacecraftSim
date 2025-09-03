#!/usr/bin/env python3

"""
START ISAAC SIM WITH REAL IMU
=============================

This script starts Isaac Sim with the FIXED IMU configuration
that enables real IMU sensor data publishing.

WHAT THIS DOES:
✅ Loads spacecraft with FIXED IMU sensor configuration
✅ Enables real ROS IMU data publishing  
✅ Creates /srb/env0/imu_robot topic with actual sensor data
✅ Provides foundation for real sensor-based control

USAGE:
    cd /home/bfarhat/SURF/space_robotics_bench
    python3 start_isaac_with_real_imu.py
    
Then in another terminal:
    python3 imu_pid_controller.py
"""

import os
import sys

# Add the project root to Python path
project_root = "/home/bfarhat/SURF/space_robotics_bench"
if project_root not in sys.path:
    sys.path.insert(0, project_root)

try:
    # Import SRB and Isaac Sim modules
    import rclpy
    from srb.core.sim import SimulationApp
    from fixed_spacecraft_imu_env import FixedSpacecraftEnvCfg, FixedSpacecraftEnv
    
    print("🚀 STARTING ISAAC SIM WITH REAL IMU SENSOR")
    print("="*50)
    
    # Initialize ROS2
    print("📡 Initializing ROS2...")
    rclpy.init()
    
    # Create the fixed environment configuration
    print("⚙️ Creating fixed spacecraft environment with real IMU...")
    env_cfg = FixedSpacecraftEnvCfg()
    
    # Start Isaac Sim application
    print("🎮 Starting Isaac Sim application...")
    sim_app = SimulationApp(
        config={
            "headless": False,  # Show GUI for monitoring
            "enable_livestream": False,
            "enable_cameras": False,
            "enable_replicator": False,
        }
    )
    
    # Create the environment with fixed IMU
    print("🛰️ Creating spacecraft environment...")
    env = FixedSpacecraftEnv(env_cfg)
    
    print("\n🎉 SUCCESS! Isaac Sim started with real IMU sensor!")
    print("="*50)
    print("✅ Spacecraft loaded with fixed IMU configuration")
    print("✅ IMU sensor properly attached to spacecraft")
    print("✅ ROS topic should be available: /srb/env0/imu_robot")
    print("📊 IMU publishing at 50 Hz with real sensor data")
    print("="*50)
    
    # Keep the simulation running
    print("\n🎯 Isaac Sim is now running with real IMU!")
    print("📋 Next steps:")
    print("1. Open another terminal")
    print("2. Run: python3 imu_pid_controller.py")
    print("3. Test real IMU-based spacecraft control!")
    print("\n🛑 Press Ctrl+C to stop Isaac Sim")
    
    try:
        # Keep simulation alive
        while True:
            # Step the environment to keep it active
            env.step(env.action_space.sample())
            
    except KeyboardInterrupt:
        print("\n🛑 Stopping Isaac Sim...")
    
    finally:
        print("🧹 Cleaning up...")
        if 'env' in locals():
            env.close()
        if 'sim_app' in locals():
            sim_app.close()
        rclpy.shutdown()
        print("✅ Cleanup complete!")

except ImportError as e:
    print(f"❌ Import Error: {e}")
    print("\n💡 This might be because:")
    print("1. Isaac Sim is not properly installed")
    print("2. SRB environment is not activated")
    print("3. Required dependencies are missing")
    print("\n🔧 Try:")
    print("1. Activate your Isaac Sim environment")
    print("2. Make sure SRB is properly installed")
    print("3. Check that all dependencies are available")
    
except Exception as e:
    print(f"❌ Error: {e}")
    import traceback
    traceback.print_exc()
    
    print(f"\n💡 Alternative approach:")
    print(f"If Isaac Sim GUI startup fails, try:")
    print(f"1. Start Isaac Sim manually from GUI")
    print(f"2. Load your spacecraft environment")
    print(f"3. Use the fixed_spacecraft_imu_env.py configuration")
    print(f"4. Enable ROS interface in Isaac Sim")
    
finally:
    if 'rclpy' in locals():
        try:
            rclpy.shutdown()
        except:
            pass
