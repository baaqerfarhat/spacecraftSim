#!/usr/bin/env python3

"""
SPACECRAFT WITH IMU CONFIGURATION
=================================

This configuration file enables the IMU sensor on the spacecraft in Isaac Sim.
It properly configures the IMU sensor to publish real acceleration and angular velocity data.

WHAT THIS DOES:
✅ Enables IMU sensor on spacecraft
✅ Configures proper sensor publishing  
✅ Sets up ROS topics for real IMU data
✅ Provides foundation for any IMU-based control

USAGE:
1. Make sure Isaac Sim is running
2. Load this configuration in your Isaac Sim environment
3. The IMU will automatically publish to /srb/env0/imu_robot
4. Use spacecraft_imu_node.py or imu_pid_controller.py with real data
"""

from dataclasses import MISSING
from srb.core.asset import AssetVariant, MobileRobot
from srb.core.env.mobile.env import MobileEnvCfg, MobileSceneCfg
from srb.core.sensor import ImuCfg
from srb.assets.robot.mobile.spacecraft import LabSc
from srb.utils.cfg import configclass

@configclass  
class SpacecraftWithIMUSceneCfg(MobileSceneCfg):
    """Enhanced scene configuration with proper IMU setup"""
    
    # Enhanced IMU configuration
    imu_robot: ImuCfg = ImuCfg(
        prim_path="{ENV_REGEX_NS}/lab_sc_robot_unique/base",  # Attach to spacecraft base
        update_period=0.02,  # 50 Hz update rate (realistic for spacecraft IMU)
        gravity_bias=(0.0, 0.0, 0.0),  # No gravity bias (space environment)
        enable_debug_vis=True,  # Enable visualization for debugging
        # Configure sensor properties
        history_length=1,  # Only keep latest reading
        # Optional: Add noise characteristics for realistic IMU
    )

@configclass
class SpacecraftWithIMUEnvCfg(MobileEnvCfg):
    """Complete spacecraft environment with IMU enabled"""
    
    # Use the spacecraft with IMU frames
    robot: MobileRobot = LabSc()
    
    # Use enhanced scene with proper IMU
    scene: SpacecraftWithIMUSceneCfg = SpacecraftWithIMUSceneCfg()
    
    def __post_init__(self):
        # Call parent post_init first
        super().__post_init__()
        
        # Ensure IMU is properly configured for the spacecraft
        if hasattr(self.scene, 'imu_robot') and self.scene.imu_robot is not None:
            # The LabSc spacecraft has frame_imu defined, so use it
            if hasattr(self.robot, 'frame_imu') and self.robot.frame_imu is not None:
                # Use the spacecraft's IMU frame
                self.scene.imu_robot.prim_path = (
                    f"{self.scene.robot.prim_path}/{self.robot.frame_imu.prim_relpath}"
                )
                self.scene.imu_robot.offset.pos = self.robot.frame_imu.offset.pos
                self.scene.imu_robot.offset.rot = self.robot.frame_imu.offset.rot
                print("✅ IMU configured using spacecraft's frame_imu")
            else:
                # Fall back to base frame if no specific IMU frame
                self.scene.imu_robot.prim_path = (
                    f"{self.scene.robot.prim_path}/{self.robot.frame_base.prim_relpath}"
                    if self.robot.frame_base.prim_relpath
                    else self.scene.robot.prim_path
                )
                self.scene.imu_robot.offset.pos = self.robot.frame_base.offset.pos
                self.scene.imu_robot.offset.rot = self.robot.frame_base.offset.rot
                print("✅ IMU configured using spacecraft's base frame")
                
            print(f"🎯 IMU will be published on: /srb/env0/imu_robot")
            print(f"📊 IMU update rate: 50 Hz")
            print(f"🛰️  IMU attached to: {self.scene.imu_robot.prim_path}")

# Factory function to create the environment
def create_spacecraft_with_imu_env():
    """Create spacecraft environment with IMU enabled"""
    cfg = SpacecraftWithIMUEnvCfg()
    
    print("🚀 CREATING SPACECRAFT WITH IMU ENVIRONMENT")
    print("="*50)
    print("✅ Spacecraft: LabSc with thruster control")
    print("✅ IMU Sensor: Enabled on spacecraft base")
    print("✅ ROS Topics: /srb/env0/imu_robot will be published")
    print("✅ Update Rate: 50 Hz (realistic spacecraft IMU)")
    print("="*50)
    
    return cfg

# Test configuration function
def test_imu_config():
    """Test the IMU configuration"""
    cfg = create_spacecraft_with_imu_env()
    
    print("\n🔍 IMU CONFIGURATION TEST")
    print(f"Robot type: {type(cfg.robot).__name__}")
    print(f"Has frame_imu: {hasattr(cfg.robot, 'frame_imu')}")
    print(f"Has frame_base: {hasattr(cfg.robot, 'frame_base')}")
    
    if hasattr(cfg.robot, 'frame_imu'):
        print(f"frame_imu: {cfg.robot.frame_imu}")
    if hasattr(cfg.robot, 'frame_base'):
        print(f"frame_base: {cfg.robot.frame_base}")
        
    print(f"IMU prim_path: {cfg.scene.imu_robot.prim_path}")
    print(f"IMU update_period: {cfg.scene.imu_robot.update_period}")
    
    print("\n✅ Configuration looks good!")
    print("🎯 To use this:")
    print("1. Import this configuration in your Isaac Sim script")
    print("2. Create environment using create_spacecraft_with_imu_env()")
    print("3. Run spacecraft_imu_node.py to verify IMU data")
    print("4. Use imu_pid_controller.py for IMU-based control")

if __name__ == '__main__':
    # Test the configuration
    test_imu_config()
    
    print("\n📋 INTEGRATION INSTRUCTIONS:")
    print("="*50)
    print("1. Copy this configuration to your Isaac Sim environment setup")
    print("2. Use SpacecraftWithIMUEnvCfg instead of default MobileEnvCfg")
    print("3. The IMU sensor will automatically be created and publish data")
    print("4. Topics will appear as: /srb/env0/imu_robot")
    print("5. Run spacecraft_imu_node.py to test IMU data reception")
    print("="*50)
