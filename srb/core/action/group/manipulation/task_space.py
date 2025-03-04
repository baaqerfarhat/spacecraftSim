from dataclasses import MISSING

import torch

from srb.core.action import (
    DifferentialInverseKinematicsActionCfg,
    OperationalSpaceControllerActionCfg,
)
from srb.core.action.action_group import ActionGroup
from srb.utils.cfg import configclass


@configclass
class InverseKinematicsActionGroup(ActionGroup):
    delta_twist: DifferentialInverseKinematicsActionCfg = MISSING  # type: ignore

    def map_cmd_to_action(self, twist: torch.Tensor, event: bool) -> torch.Tensor:
        return (
            twist if self.delta_twist.controller.command_type == "pose" else twist[:3]
        )


@configclass
class OperationalSpaceControlActionGroup(ActionGroup):
    delta_twist: OperationalSpaceControllerActionCfg = MISSING  # type: ignore

    def map_cmd_to_action(self, twist: torch.Tensor, event: bool) -> torch.Tensor:
        # TODO[mid]: Map teleop actions based on the impedance mode of the OSC
        raise NotImplementedError
