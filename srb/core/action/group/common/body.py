import torch

from srb.core.action.action_group import ActionGroup
from srb.core.action.term import BodyVelocityActionCfg
from srb.utils.cfg import configclass


@configclass
class BodyVelocityActionGroup(ActionGroup):
    body_vel: BodyVelocityActionCfg = BodyVelocityActionCfg(asset_name="robot")

    def map_cmd_to_action(self, twist: torch.Tensor, event: bool) -> torch.Tensor:
        return twist
