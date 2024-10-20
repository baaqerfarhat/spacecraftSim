from os import path

from omni.isaac.lab.utils import configclass

import space_robotics_bench.core.sim as sim_utils
from space_robotics_bench.paths import SRB_ASSETS_DIR_SRB_OBJECT


@configclass
class SampleTubeCfg(sim_utils.UsdFileCfg):
    usd_path = path.join(SRB_ASSETS_DIR_SRB_OBJECT, "sample_tube", "sample_tube.usdc")
