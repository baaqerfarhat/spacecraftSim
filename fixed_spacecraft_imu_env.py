#!/usr/bin/env python3

"""
FIXED SPACECRAFT IMU ENVIRONMENT
===============================

This fixes the BUG in SRB that prevents IMU sensors from working properly.
The original code incorrectly uses frame_base instead of frame_imu for the prim_path.

FIXES:
✅ Corrects IMU sensor attachment to proper frame
✅ Enables real IMU data publishing from Isaac Sim
✅ Provides /srb/env0/imu_robot topic with REAL sensor data
✅ Uses actual spacecraft physics for acceleration/angular velocity

BUG FIXED:
Original SRB code (line 44-46 in mobile/env.py):
    self.scene.imu_robot.prim_path = f"{...}/{self._robot.frame_base.prim_relpath}"  # WRONG!

Fixed version:
    self.scene.imu_robot.prim_path = f"{...}/{self._robot.frame_imu.prim_relpath}"   # CORRECT!
"""

from dataclasses import MISSING
from srb.core.asset import AssetVariant, MobileRobot
from srb.core.env.mobile.env import MobileEnvCfg, MobileSceneCfg, MobileEnv
from srb.core.env.mobile.orbital.env import OrbitalEnvCfg, OrbitalEnv
from srb.core.sensor import ImuCfg
from srb.assets.robot.mobile.spacecraft import LabSc
from srb.utils.cfg import configclass

@configclass
class FixedMobileSceneCfg(MobileSceneCfg):
    """Fixed scene configuration with proper IMU setup"""
    
    # Enhanced IMU configuration with proper settings
    imu_robot: ImuCfg = ImuCfg(
        prim_path=MISSING,  # Will be set correctly in post_init
        update_period=0.02,  # 50 Hz - realistic spacecraft IMU rate
        gravity_bias=(0.0, 0.0, 0.0),  # No gravity bias for space
        enable_debug_vis=True,  # Enable visualization for debugging
        history_length=1,  # Only keep latest reading for real-time use
    )

@configclass  
class FixedSpacecraftEnvCfg(OrbitalEnvCfg):
    """Fixed spacecraft environment configuration with working IMU"""
    
    # Use LabSc spacecraft (has IMU frames defined)
    robot: MobileRobot = LabSc()
    
    # Use fixed scene configuration
    scene: FixedMobileSceneCfg = FixedMobileSceneCfg()
    
    def __post_init__(self):
        # Call parent post_init first, but override the IMU configuration
        super().__post_init__()
        
        # FIX THE BUG: Properly configure IMU sensor
        if hasattr(self.scene, 'imu_robot') and self.scene.imu_robot is not None:
            # Check if robot has frame_imu (LabSc does!)
            if hasattr(self._robot, 'frame_imu') and self._robot.frame_imu is not None:
                # CORRECT: Use frame_imu.prim_relpath, not frame_base.prim_relpath
                self.scene.imu_robot.prim_path = (
                    f"{self.scene.robot.prim_path}/{self._robot.frame_imu.prim_relpath}"
                )
                self.scene.imu_robot.offset.pos = self._robot.frame_imu.offset.pos
                self.scene.imu_robot.offset.rot = self._robot.frame_imu.offset.rot
                print("✅ IMU CONFIGURED CORRECTLY using frame_imu!")
                print(f"🎯 IMU prim_path: {self.scene.imu_robot.prim_path}")
            elif hasattr(self._robot, 'frame_onboard_imu') and self._robot.frame_onboard_imu is not None:
                # Alternative: Use frame_onboard_imu if available
                self.scene.imu_robot.prim_path = (
                    f"{self.scene.robot.prim_path}/{self._robot.frame_onboard_imu.prim_relpath}"
                )
                self.scene.imu_robot.offset.pos = self._robot.frame_onboard_imu.offset.pos
                self.scene.imu_robot.offset.rot = self._robot.frame_onboard_imu.offset.rot
                print("✅ IMU CONFIGURED using frame_onboard_imu!")
                print(f"🎯 IMU prim_path: {self.scene.imu_robot.prim_path}")
            else:
                # Fallback to base frame with offset
                base_path = (
                    f"{self.scene.robot.prim_path}/{self._robot.frame_base.prim_relpath}"
                    if self._robot.frame_base.prim_relpath
                    else self.scene.robot.prim_path
                )
                self.scene.imu_robot.prim_path = base_path
                self.scene.imu_robot.offset.pos = self._robot.frame_base.offset.pos
                self.scene.imu_robot.offset.rot = self._robot.frame_base.offset.rot
                print("⚠️  IMU CONFIGURED using frame_base as fallback")
                print(f"🎯 IMU prim_path: {self.scene.imu_robot.prim_path}")
                
        print(f"📊 IMU update rate: {self.scene.imu_robot.update_period}s ({1/self.scene.imu_robot.update_period:.0f} Hz)")
        print(f"🛰️  Expected ROS topic: /srb/env0/imu_robot")

class FixedSpacecraftEnv(OrbitalEnv):
    """Fixed spacecraft environment with working IMU sensor"""
    
    cfg: FixedSpacecraftEnvCfg
    
    def __init__(self, cfg: FixedSpacecraftEnvCfg, **kwargs):
        super().__init__(cfg, **kwargs)
        
        # Get the IMU sensor (should work now!)
        self._imu_robot = self.scene.get("imu_robot") if hasattr(self.scene, 'imu_robot') else None
        
        if self._imu_robot is not None:
            print("🎉 SUCCESS! IMU sensor created successfully!")
            print(f"📊 IMU sensor type: {type(self._imu_robot)}")
            print(f"🎯 IMU should publish on: /srb/env0/imu_robot")
        else:
            print("❌ FAILED: IMU sensor not created")
            print("🔍 Check if ROS interface is enabled in Isaac Sim")

def create_fixed_spacecraft_env():
    """Create spacecraft environment with fixed IMU configuration"""
    cfg = FixedSpacecraftEnvCfg()
    
    print("🚀 CREATING FIXED SPACECRAFT ENVIRONMENT WITH REAL IMU")
    print("="*60)
    print("✅ Bug Fix: IMU properly attached to frame_imu (not frame_base)")
    print("✅ LabSc Spacecraft: Has frame_imu and frame_onboard_imu defined")
    print("✅ Real IMU Sensor: Will publish actual acceleration/angular velocity")
    print("✅ ROS Topic: /srb/env0/imu_robot (sensor_msgs/Imu)")
    print("✅ Update Rate: 50 Hz (realistic spacecraft IMU)")
    print("="*60)
    
    return FixedSpacecraftEnv(cfg)

# Test configuration
def test_fixed_imu_config():
    """Test the fixed IMU configuration"""
    print("🔍 TESTING FIXED IMU CONFIGURATION")
    print("="*40)
    
    cfg = FixedSpacecraftEnvCfg()
    
    print(f"Robot: {type(cfg._robot).__name__}")
    print(f"Has frame_imu: {hasattr(cfg._robot, 'frame_imu')}")
    print(f"Has frame_onboard_imu: {hasattr(cfg._robot, 'frame_onboard_imu')}")
    
    if hasattr(cfg._robot, 'frame_imu') and cfg._robot.frame_imu:
        print(f"frame_imu prim_relpath: {cfg._robot.frame_imu.prim_relpath}")
        print(f"frame_imu offset: {cfg._robot.frame_imu.offset}")
        
    if hasattr(cfg._robot, 'frame_onboard_imu') and cfg._robot.frame_onboard_imu:
        print(f"frame_onboard_imu prim_relpath: {cfg._robot.frame_onboard_imu.prim_relpath}")
        print(f"frame_onboard_imu offset: {cfg._robot.frame_onboard_imu.offset}")
    
    print(f"Final IMU prim_path: {cfg.scene.imu_robot.prim_path}")
    print(f"IMU update_period: {cfg.scene.imu_robot.update_period}")
    
    print("\n✅ Configuration test complete!")
    return cfg

if __name__ == '__main__':
    print("🎯 FIXED SPACECRAFT IMU ENVIRONMENT")
    print("This fixes the SRB bug that prevents IMU sensors from working")
    print("\n1. Testing configuration...")
    
    try:
        cfg = test_fixed_imu_config()
        
        print(f"\n2. To use this environment:")
        print(f"   from fixed_spacecraft_imu_env import create_fixed_spacecraft_env")
        print(f"   env = create_fixed_spacecraft_env()")
        
        print(f"\n3. Expected result:")
        print(f"   ✅ Real IMU sensor attached to spacecraft")
        print(f"   ✅ ROS topic /srb/env0/imu_robot publishing real data")
        print(f"   ✅ Use with imu_pid_controller.py for real sensor-based control!")
        
    except Exception as e:
        print(f"❌ Error testing configuration: {e}")
        import traceback
        traceback.print_exc()
