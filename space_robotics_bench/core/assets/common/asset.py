from pydantic import BaseModel

from space_robotics_bench.core.assets import AssetBaseCfg


class AssetCfg(BaseModel):
    ## Model
    asset_cfg: AssetBaseCfg
