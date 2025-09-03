from srb.core.env.common.extension.visual import VisualExtCfg
from srb.core.sensor import CameraCfg, PinholeCameraCfg
from srb.utils.cfg import configclass
from srb.utils.math import rpy_to_quat  # noqa: F401

from .env import OrbitalEnvCfg


@configclass
class OrbitalEnvVisualExtCfg(VisualExtCfg):
    def wrap(self, env_cfg: OrbitalEnvCfg):
        # Get the actual robot prim path from the robot's asset config
        robot_prim_path = env_cfg._robot.asset_cfg.prim_path
        
        self.cameras_cfg = {
            # "cam_scene": CameraCfg(
            #     prim_path=f"{env_cfg._robot.asset_cfg.prim_path}{('/' + env_cfg._robot.frame_base.prim_relpath) if env_cfg._robot.frame_base.prim_relpath else ''}/camera_scene",
            #     offset=CameraCfg.OffsetCfg(
            #         convention="world",
            #         pos=(-2.5, 0.0, 2.5),
            #         rot=rpy_to_quat(0.0, 45.0, 0.0),
            #     ),
            #     spawn=PinholeCameraCfg(
            #         clipping_range=(0.25, 250.0 + 0.25),
            #     ),
            # ),
            "cam_onboard": CameraCfg(
                prim_path=f"{robot_prim_path}/camera_onboard_unique",
                update_period=1.0 / 50.0,  # 50 Hz camera rate - higher performance
                width=32,  # Even smaller resolution for maximum speed
                height=32,
                data_types=("rgb",),  # RGB only
                offset=CameraCfg.OffsetCfg(
                    convention="world",
                    pos=env_cfg._robot.frame_onboard_camera.offset.pos,
                    rot=env_cfg._robot.frame_onboard_camera.offset.rot,
                ),
                spawn=PinholeCameraCfg(
                    focal_length=25.0,
                    horizontal_aperture=12.0,
                    clipping_range=(0.1, 200.0 + 0.1),
                ),
            ),
        }

        super().wrap(env_cfg)
