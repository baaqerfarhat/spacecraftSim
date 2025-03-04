from dataclasses import MISSING

from srb import assets
from srb.core.asset import AerialManipulator, Articulation, AssetVariant
from srb.core.env import ViewerCfg
from srb.core.env.mobile.aerial.env import (
    AerialEnv,
    AerialEnvCfg,
    AerialEventCfg,
    AerialSceneCfg,
)
from srb.core.manager import EventTermCfg
from srb.utils.cfg import configclass


@configclass
class AerialManipulationSceneCfg(AerialSceneCfg):
    env_spacing = 16.0


@configclass
class AerialManipulationEventCfg(AerialEventCfg):
    randomize_robot_state: EventTermCfg | None = None


@configclass
class AerialManipulationEnvCfg(AerialEnvCfg):
    ## Assets
    robot: AerialManipulator | AssetVariant = assets.GenericAerialManipulator(
        mobile_base=assets.Ingenuity(), manipulator=assets.Franka()
    )
    _robot: AerialManipulator = MISSING  # type: ignore

    ## Scene
    scene: AerialManipulationSceneCfg = AerialManipulationSceneCfg()

    ## Events
    events: AerialManipulationEventCfg = AerialManipulationEventCfg()

    ## Time
    env_rate: float = 1.0 / 150.0
    agent_rate: float = 1.0 / 50.0

    ## Viewer
    viewer: ViewerCfg = ViewerCfg(
        eye=(10.0, -10.0, 10.0), lookat=(0.0, 0.0, 0.0), origin_type="env"
    )

    def __post_init__(self):
        super().__post_init__()


class AerialManipulationEnv(AerialEnv):
    cfg: AerialManipulationEnvCfg

    def __init__(self, cfg: AerialManipulationEnvCfg, **kwargs):
        super().__init__(cfg, **kwargs)

        ## Get scene assets
        self._manipulator: Articulation = self.scene["manipulator"]
