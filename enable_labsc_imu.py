#!/usr/bin/env python3

"""
ENABLE LABSC IMU SENSOR IN ISAAC SIM
===================================

This script creates the proper environment configuration to enable 
the real IMU sensor on the LabSc spacecraft that publishes to ROS.

RESEARCH FINDINGS:
✅ LabSc spacecraft HAS IMU frames (frame_imu, frame_onboard_imu)
✅ ROS interface DOES support IMU publishing (lines 850-865 in ros.py)
✅ Expected topic: /srb/env0/imu_robot
✅ The issue: IMU sensor not instantiated in scene

SOLUTION:
✅ Create proper environment with IMU sensor enabled
✅ Use LabSc spacecraft's existing IMU frames
✅ Enable ROS interface IMU publishing
✅ Get real acceleration/angular velocity from Isaac Sim physics
"""

import os
import sys

# Add the project to Python path
sys.path.insert(0, '/home/bfarhat/SURF/space_robotics_bench')

def create_labsc_imu_environment():
    """Create the proper environment configuration for LabSc with IMU"""
    
    print("🔧 CREATING LABSC IMU ENVIRONMENT")
    print("="*50)
    
    try:
        # Import required modules
        from srb.core.env.mobile.orbital.env import OrbitalEnvCfg, OrbitalEnv
        from srb.core.env.mobile.env import MobileSceneCfg
        from srb.core.sensor import ImuCfg
        from srb.assets.robot.mobile.spacecraft import LabSc
        from srb.utils.cfg import configclass
        from dataclasses import MISSING
        
        @configclass
        class LabScIMUSceneCfg(MobileSceneCfg):
            """Scene configuration with properly enabled IMU for LabSc"""
            
            # PROPERLY configure IMU for LabSc spacecraft
            imu_robot: ImuCfg = ImuCfg(
                prim_path="{ENV_REGEX_NS}/lab_sc_robot_unique/base",  # LabSc spacecraft path
                update_period=0.02,  # 50 Hz - standard spacecraft IMU rate
                gravity_bias=(0.0, 0.0, 0.0),  # No gravity bias (space environment)
                # Use the offset from LabSc frame_imu definition
                offset=LabSc().frame_imu.offset,  # Use LabSc's IMU frame offset!
                history_length=1,  # Real-time data only
            )
        
        @configclass
        class LabScIMUEnvCfg(OrbitalEnvCfg):
            """LabSc orbital environment with IMU enabled"""
            
            # Use LabSc spacecraft
            robot = LabSc()
            
            # Use scene with properly configured IMU
            scene: LabScIMUSceneCfg = LabScIMUSceneCfg()
            
            def __post_init__(self):
                # Call parent initialization
                super().__post_init__()
                
                # CORRECT IMU configuration using LabSc's IMU frame
                if hasattr(self.scene, 'imu_robot') and self.scene.imu_robot is not None:
                    # Use LabSc's frame_imu for proper attachment
                    self.scene.imu_robot.prim_path = (
                        f"{self.scene.robot.prim_path}/{self._robot.frame_imu.prim_relpath}"
                    )
                    # Use LabSc's IMU frame offset
                    self.scene.imu_robot.offset = self._robot.frame_imu.offset
                    
                    print("✅ IMU PROPERLY CONFIGURED!")
                    print(f"🛰️  Spacecraft: {type(self._robot).__name__}")
                    print(f"🎯 IMU prim_path: {self.scene.imu_robot.prim_path}")
                    print(f"📊 IMU offset: {self.scene.imu_robot.offset}")
                    print(f"⚡ Update rate: {1/self.scene.imu_robot.update_period:.0f} Hz")
                    print(f"📡 Expected ROS topic: /srb/env0/imu_robot")
        
        # Create and return the environment configuration
        cfg = LabScIMUEnvCfg()
        return cfg
        
    except ImportError as e:
        print(f"❌ Import error: {e}")
        print("💡 This script must be run from Isaac Sim environment")
        print("   or with SRB properly configured")
        return None
    except Exception as e:
        print(f"❌ Error creating environment: {e}")
        import traceback
        traceback.print_exc()
        return None

def create_imu_startup_script():
    """Create Isaac Sim startup script with IMU enabled"""
    
    script_content = '''#!/usr/bin/env python3

"""
Isaac Sim Startup Script with LabSc IMU
=======================================

This script starts Isaac Sim with the LabSc spacecraft and properly
configured IMU sensor that publishes real sensor data to ROS.
"""

import sys
sys.path.append('/home/bfarhat/SURF/space_robotics_bench')

from enable_labsc_imu import create_labsc_imu_environment
from srb.core.env.mobile.orbital.env import OrbitalEnv
import rclpy

def main():
    print("🚀 STARTING ISAAC SIM WITH LABSC IMU")
    print("="*50)
    
    # Initialize ROS
    rclpy.init()
    
    # Create environment with IMU enabled
    cfg = create_labsc_imu_environment()
    if cfg is None:
        print("❌ Failed to create environment")
        return
    
    # Create the environment
    env = OrbitalEnv(cfg)
    
    print("✅ Isaac Sim started with LabSc IMU!")
    print("📡 IMU should be publishing on: /srb/env0/imu_robot")
    print("🎯 Use 'ros2 topic echo /srb/env0/imu_robot' to verify")
    
    try:
        # Keep simulation running
        print("🔄 Simulation running... Press Ctrl+C to stop")
        while True:
            env.step(env.action_space.sample())
    except KeyboardInterrupt:
        print("🛑 Stopping simulation...")
    finally:
        env.close()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
'''
    
    with open('/home/bfarhat/SURF/space_robotics_bench/start_labsc_imu.py', 'w') as f:
        f.write(script_content)
    
    print("✅ Created Isaac Sim startup script: start_labsc_imu.py")

def main():
    print("🛰️ LABSC IMU ENABLER")
    print("="*50)
    print("Based on codebase research:")
    print("✅ LabSc has unique IMU frames (unlike other robots)")
    print("✅ ROS interface supports IMU publishing")
    print("✅ Isaac Sim can publish real sensor data")
    print("="*50)
    
    # Test environment creation
    cfg = create_labsc_imu_environment()
    
    if cfg is not None:
        print("\n🎉 SUCCESS! Environment configuration created")
        print("="*50)
        
        # Create startup script
        create_imu_startup_script()
        
        print(f"\n📋 NEXT STEPS:")
        print(f"1. 🚀 Start Isaac Sim with IMU:")
        print(f"      python3 start_labsc_imu.py")
        print(f"")
        print(f"2. 🔍 Verify IMU topic (in another terminal):")
        print(f"      ros2 topic list | grep imu")
        print(f"      ros2 topic echo /srb/env0/imu_robot --once")
        print(f"")
        print(f"3. 🎯 Use real IMU control:")
        print(f"      python3 imu_pid_controller.py")
        print(f"")
        print(f"✨ You'll get REAL acceleration data from Isaac Sim physics!")
        
    else:
        print("\n❌ Failed to create environment")
        print("💡 Make sure SRB environment is properly configured")

if __name__ == '__main__':
    main()
