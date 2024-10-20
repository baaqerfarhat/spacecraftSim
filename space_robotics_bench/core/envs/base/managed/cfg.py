from omni.isaac.lab.envs import ManagerBasedRLEnvCfg
from omni.isaac.lab.utils import configclass


@configclass
class BaseEnvManagedCfg(ManagerBasedRLEnvCfg):
    """
    Extended version of :class:`omni.isaac.lab.envs.ManagerBasedRLEnvCfg`.
    """

    # Disable UI window by default
    ui_window_class_type: type | None = None
