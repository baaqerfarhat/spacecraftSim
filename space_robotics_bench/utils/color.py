from typing import Tuple

import space_robotics_bench.core.envs as env_utils
import space_robotics_bench.core.sim as sim_utils


def contrastive_color_from_env_cfg(
    env_cfg: env_utils.EnvironmentConfig,
) -> Tuple[float, float, float]:
    match env_cfg.scenario:
        case env_utils.Scenario.ASTEROID | env_utils.Scenario.MOON:
            return (0.8, 0.8, 0.8)
        case (
            env_utils.Scenario.EARTH
            | env_utils.Scenario.MARS
            | env_utils.Scenario.ORBIT
        ):
            return (0.1, 0.1, 0.1)
        case _:
            return (0.7071, 0.7071, 0.7071)


def preview_surface_from_env_cfg(
    env_cfg: env_utils.EnvironmentConfig,
) -> sim_utils.PreviewSurfaceCfg:
    return sim_utils.PreviewSurfaceCfg(
        diffuse_color=contrastive_color_from_env_cfg(env_cfg),
    )
