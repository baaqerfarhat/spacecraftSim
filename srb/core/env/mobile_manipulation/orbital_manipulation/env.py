from dataclasses import MISSING

from srb import assets
from srb.core.asset import Articulation, AssetVariant, OrbitalManipulator
from srb.core.env import ViewerCfg
from srb.core.env.mobile.orbital.env import (
    OrbitalEnv,
    OrbitalEnvCfg,
    OrbitalEventCfg,
    OrbitalSceneCfg,
)
from srb.core.manager import EventTermCfg
from srb.utils.cfg import configclass


@configclass
class OrbitalManipulationSceneCfg(OrbitalSceneCfg):
    pass


@configclass
class OrbitalManipulationEventCfg(OrbitalEventCfg):
    randomize_robot_state: EventTermCfg | None = None


@configclass
class OrbitalManipulationEnvCfg(OrbitalEnvCfg):
    ## Assets
    robot: OrbitalManipulator | AssetVariant = assets.GenericOrbitalManipulator(
        mobile_base=assets.Cubesat(), manipulator=assets.Franka()
    )
    _robot: OrbitalManipulator = MISSING  # type: ignore

    ## Scene
    scene: OrbitalManipulationSceneCfg = OrbitalManipulationSceneCfg()

    ## Events
    events: OrbitalManipulationEventCfg = OrbitalManipulationEventCfg()

    ## Time
    env_rate: float = 1.0 / 150.0
    agent_rate: float = 1.0 / 50.0

    ## Viewer
    viewer: ViewerCfg = ViewerCfg(
        eye=(10.0, -10.0, 10.0), lookat=(0.0, 0.0, 0.0), origin_type="env"
    )

    def __post_init__(self):
        super().__post_init__()


class OrbitalManipulationEnv(OrbitalEnv):
    cfg: OrbitalManipulationEnvCfg

    def __init__(self, cfg: OrbitalManipulationEnvCfg, **kwargs):
        super().__init__(cfg, **kwargs)

        ## Get scene assets
        self._manipulator: Articulation = self.scene["manipulator"]
