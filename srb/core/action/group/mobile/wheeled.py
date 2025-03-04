from dataclasses import MISSING

import torch

from srb.core.action.action_group import ActionGroup
from srb.core.action.term import WheeledRoverDriveActionCfg
from srb.utils.cfg import configclass

# TODO[low]: Implement diff-drive and skid-steer drive


@configclass
class WheeledRoverActionGroup(ActionGroup):
    cmd_vel: WheeledRoverDriveActionCfg = MISSING  # type: ignore

    def map_cmd_to_action(self, twist: torch.Tensor, event: bool) -> torch.Tensor:
        return twist[:2]
