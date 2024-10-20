from typing import Any

from space_robotics_bench.core.actions import WheeledRoverActionGroupCfg
from space_robotics_bench.core.assets import FrameCfg

from . import RobotCfg


class MobileRobotCfg(RobotCfg):
    ## Actions
    action_cfg: Any


class WheeledRoverCfg(MobileRobotCfg):
    ## Actions
    action_cfg: WheeledRoverActionGroupCfg

    ## Frames
    frame_camera_front: FrameCfg

    ## Joints
    regex_drive_joints: str
    regex_steer_joints: str
