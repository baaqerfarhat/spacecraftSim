from typing import Any

from space_robotics_bench.core.actions import MultiCopterActionGroupCfg
from space_robotics_bench.core.assets import FrameCfg

from . import RobotCfg


class AerialRobotCfg(RobotCfg):
    ## Actions
    action_cfg: Any


class MultiCopterCfg(AerialRobotCfg):
    class Config:
        arbitrary_types_allowed = True  # Due to MultiCopterActionGroupCfg

    ## Actions
    action_cfg: MultiCopterActionGroupCfg

    ## Frames
    frame_camera_bottom: FrameCfg

    ## Links
    regex_links_rotors: str

    ## Joints
    regex_joints_rotors: str
