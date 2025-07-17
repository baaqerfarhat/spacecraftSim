from srb.core.asset import Object, RigidObjectCfg
from srb.core.sim import UsdFileAssetCfg  # adjust if named differently in your SRB version

class Apophis(Object):
    asset_cfg: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/apophis",
        spawn=UsdFileAssetCfg(  # replace with correct config class if needed
            usd_path="/home/bfarhat/SURF/space_robotics_bench/srb/assets/object/apophis.usd",
            scale=(1.0, 1.0, 1.0),
        ),
    )
