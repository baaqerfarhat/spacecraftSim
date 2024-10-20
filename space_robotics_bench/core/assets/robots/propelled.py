from space_robotics_bench.core.actions import PropelledRobotTaskSpaceActionCfg
from space_robotics_bench.core.assets import FrameCfg

from . import RobotCfg


class PropelledRobotCfg(RobotCfg):
    ## Actions
    action_cfg: PropelledRobotTaskSpaceActionCfg


class MultiCopterCfg(PropelledRobotCfg):
    ## Frames
    frame_camera_bottom: FrameCfg

    # ## Links
    # regex_links_wheels: str

    # ## Joints
    # regex_joints_wheels: str
