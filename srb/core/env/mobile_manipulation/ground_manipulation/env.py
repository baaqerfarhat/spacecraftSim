from dataclasses import MISSING

from srb import assets
from srb.core.asset import Articulation, AssetVariant, GroundManipulator
from srb.core.env import ViewerCfg
from srb.core.env.mobile.ground.env import (
    GroundEnv,
    GroundEnvCfg,
    GroundEventCfg,
    GroundSceneCfg,
)
from srb.core.manager import EventTermCfg
from srb.utils.cfg import configclass


@configclass
class GroundManipulationSceneCfg(GroundSceneCfg):
    env_spacing = 16.0


@configclass
class GroundManipulationEventCfg(GroundEventCfg):
    randomize_robot_state: EventTermCfg | None = None


@configclass
class GroundManipulationEnvCfg(GroundEnvCfg):
    ## Assets
    robot: GroundManipulator | AssetVariant = assets.GenericGroundManipulator(
        mobile_base=assets.AnymalD(), manipulator=assets.Franka()
    )
    _robot: GroundManipulator = MISSING  # type: ignore

    ## Scene
    scene: GroundManipulationSceneCfg = GroundManipulationSceneCfg()

    ## Events
    events: GroundManipulationEventCfg = GroundManipulationEventCfg()

    ## Time
    env_rate: float = 1.0 / 150.0
    agent_rate: float = 1.0 / 50.0

    ## Viewer
    viewer: ViewerCfg = ViewerCfg(
        eye=(10.0, -10.0, 10.0), lookat=(0.0, 0.0, 0.0), origin_type="env"
    )

    def __post_init__(self):
        super().__post_init__()


class GroundManipulationEnv(GroundEnv):
    cfg: GroundManipulationEnvCfg

    def __init__(self, cfg: GroundManipulationEnvCfg, **kwargs):
        super().__init__(cfg, **kwargs)

        ## Get scene assets
        self._manipulator: Articulation = self.scene["manipulator"]
