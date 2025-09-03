from dataclasses import MISSING

from srb.core.asset import AssetVariant, Humanoid, MobileRobot
from srb.core.env import BaseEventCfg, BaseSceneCfg, DirectEnv, DirectEnvCfg
from srb.core.marker import RED_ARROW_X_MARKER_CFG
from srb.core.sensor import Imu, ImuCfg
from srb.utils.cfg import configclass


@configclass
class MobileSceneCfg(BaseSceneCfg):
    imu_robot: ImuCfg = ImuCfg(
        prim_path=MISSING,  # type: ignore
        update_period=1.0 / 500.0,  # 500 Hz IMU rate - ultra high performance
        # Space mode: subtract Earth gravity so IMU reports 0 when static in space
        gravity_bias=(0.0, 0.0, -9.81),
        visualizer_cfg=RED_ARROW_X_MARKER_CFG.replace(  # type: ignore
            prim_path="/Visuals/imu_robot/lin_acc"
        ),
    )


@configclass
class MobileEventCfg(BaseEventCfg):
    pass


@configclass
class MobileEnvCfg(DirectEnvCfg):
    ## Assets
    robot: MobileRobot | Humanoid | AssetVariant = MISSING  # type: ignore
    _robot: MobileRobot | Humanoid = MISSING  # type: ignore

    ## Scene
    scene: MobileSceneCfg = MobileSceneCfg()

    ## Events
    events: MobileEventCfg = MobileEventCfg()

    def __post_init__(self):
        super().__post_init__()

        # Sensor: Robot IMU (only configure if not disabled)
        if hasattr(self.scene, 'imu_robot') and self.scene.imu_robot is not None:
            # Bind IMU directly to robot root which carries RigidBodyAPI
            # This avoids failures when frame_base is a pure Xform without physics
            self.scene.imu_robot.prim_path = self.scene.robot.prim_path


class MobileEnv(DirectEnv):
    cfg: MobileEnvCfg

    def __init__(self, cfg: MobileEnvCfg, **kwargs):
        super().__init__(cfg, **kwargs)

        ## Get scene assets
        # Only get IMU if it's configured
        self._imu_robot: Imu | None = self.scene.get("imu_robot") if hasattr(self.scene, 'imu_robot') and self.scene.imu_robot is not None else None
        # If IMU config exists but sensor wasn't instantiated, create it now and store it
        if self._imu_robot is None and hasattr(self.scene, 'imu_robot') and self.scene.imu_robot is not None:
            try:
                self.scene["imu_robot"] = Imu(self.scene.imu_robot)
                self._imu_robot = self.scene.get("imu_robot")
            except Exception:
                self._imu_robot = None
