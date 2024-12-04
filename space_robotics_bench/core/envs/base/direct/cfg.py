from os import path

import gymnasium
from omni.isaac.lab.envs import DirectRLEnvCfg
from omni.isaac.lab.utils import configclass

import space_robotics_bench.core.envs as env_utils
from space_robotics_bench.paths import SRB_CONFIG_DIR


@configclass
class BaseEnvCfg(DirectRLEnvCfg):
    """
    Extended version of :class:`omni.isaac.lab.envs.DirectRLEnvCfg`.
    """

    ## Updated defaults
    # Disable UI window by default
    ui_window_class_type: type | None = None
    # Redundant: spaces are automatically extracted
    num_actions: int = 0
    # Redundant: spaces are automatically extracted
    num_observations: int = 0

    ## Environment
    env_cfg: env_utils.EnvironmentConfig = env_utils.EnvironmentConfig.extract(
        cfg_path=path.join(SRB_CONFIG_DIR, "env.yaml")
    )

    ## Misc
    # Flag that disables the timeout for the environment
    enable_truncation: bool = True

    ## Ugly hack to gain compatibility with new Isaac Lab
    # TODO: Fix in a better way
    action_space = gymnasium.spaces.Box(low=-1.0, high=1.0, shape=(1,))
    observation_space = gymnasium.spaces.Box(low=-1.0, high=1.0, shape=(1,))
