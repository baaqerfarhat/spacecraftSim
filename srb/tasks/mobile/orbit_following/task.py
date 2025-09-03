from dataclasses import MISSING
from typing import Sequence

import torch

from srb._typing.step_return import StepReturn
from srb import assets
from srb.core.action import ThrustAction
from srb.core.asset import RigidObjectCollection, RigidObjectCollectionCfg, RigidObjectCfg
from srb.core.env.mobile import (
    OrbitalEnv,
    OrbitalEnvCfg,
    OrbitalSceneCfg,
    OrbitalEventCfg,
)
from srb.core.manager import EventTermCfg
from srb.core.sim import UsdFileCfg
from srb.utils.cfg import configclass
from srb.utils.math import matrix_from_quat, rotmat_to_rot6d


@configclass
class EventCfg(OrbitalEventCfg):
    # Use defaults from base; we just need a non-None events container
    pass

@configclass
class SceneCfg(OrbitalSceneCfg):
    env_spacing: float = 50.0
    objs: RigidObjectCollectionCfg = RigidObjectCollectionCfg(rigid_objects=MISSING)  # type: ignore


@configclass
class TaskCfg(OrbitalEnvCfg):
    scene: SceneCfg = SceneCfg()
    # Use LabSc spacecraft (8-thruster configuration) instead of default Cubesat
    robot = assets.LabSc()
    # Provide a proper events config to avoid None in base setup
    # (base env expects to set fields like randomize_skydome_orientation)
    # Define a minimal EventCfg derived from OrbitalEventCfg
    
    # Backwards-compatible placeholder; replaced just below with EventCfg()
    events: 'EventCfg' = None  # type: ignore
    episode_length_s: float = 60.0  # Increased from 30.0 to allow more time for learning
    is_finite_horizon: bool = True
    env_rate: float = 1.0 / 50.0

    # Orbit params
    orbit_diameter_m: float = 100.0
    tangential_speed_mps: float = 2.0
    
    # Curriculum learning parameters
    start_orbit_diameter_m: float = 50.0  # Start with smaller orbit
    target_orbit_diameter_m: float = 100.0  # Target orbit size
    curriculum_steps: int = 500000  # Steps to reach target orbit size

    def __post_init__(self):
        # Ensure events is a valid config object
        if self.events is None:  # type: ignore
            self.events = EventCfg()  # type: ignore
        # Ensure asteroid exists and is named apophis_unique with proper typed config
        self.scene.objs.rigid_objects = {
            "apophis": RigidObjectCfg(
                prim_path="{ENV_REGEX_NS}/apophis_unique",
                spawn=UsdFileCfg(
                    usd_path="/root/ws/srb/assets/object/apophis_accurate.usdz",
                    scale=(1.0, 1.0, 1.0),
                ),
                init_state=RigidObjectCfg.InitialStateCfg(
                    pos=(0.0, 0.0, 0.0),
                    rot=(1.0, 0.0, 0.0, 0.0),
                ),
            )
        }
        super().__post_init__()


class Task(OrbitalEnv):
    cfg: TaskCfg

    def __init__(self, cfg: TaskCfg, **kwargs):
        super().__init__(cfg, **kwargs)
        self._objs: RigidObjectCollection = self.scene["objs"]
        
        # Curriculum learning: start with smaller orbit and gradually increase
        self._curriculum_progress = 0.0  # 0.0 = start, 1.0 = target
        self._current_orbit_diameter = cfg.start_orbit_diameter_m
        self._radius = max(1e-3, self._current_orbit_diameter * 0.5)
        self._omega = cfg.tangential_speed_mps / self._radius
        
        # Track training progress for curriculum
        self._total_timesteps = 0
    
    def _update_curriculum(self):
        """Update orbit parameters based on training progress"""
        if self._total_timesteps < self.cfg.curriculum_steps:
            self._curriculum_progress = self._total_timesteps / self.cfg.curriculum_steps
        else:
            self._curriculum_progress = 1.0
            
        # Interpolate between start and target orbit size
        self._current_orbit_diameter = (
            self.cfg.start_orbit_diameter_m + 
            self._curriculum_progress * (self.cfg.target_orbit_diameter_m - self.cfg.start_orbit_diameter_m)
        )
        self._radius = max(1e-3, self._current_orbit_diameter * 0.5)
        self._omega = self.cfg.tangential_speed_mps / self._radius
        
        # Update total timesteps
        self._total_timesteps += 1

    def _reset_idx(self, env_ids: Sequence[int]):
        super()._reset_idx(env_ids)
        # Place robot on-circle in XY plane relative to asteroid
        n = len(env_ids)
        center = self._objs.data.object_com_pos_w[env_ids, 0]
        theta0 = torch.zeros((n,), device=self.device)
        pos = torch.zeros((n, 3), device=self.device)
        pos[:, 0] = center[:, 0] + self._radius * torch.cos(theta0)
        pos[:, 1] = center[:, 1] + self._radius * torch.sin(theta0)
        pos[:, 2] = center[:, 2]
        self._robot.data.root_pos_w[env_ids] = pos
        self._robot.data.root_lin_vel_w[env_ids] = 0.0
        self._robot.data.root_ang_vel_w[env_ids] = 0.0

    def extract_step_return(self) -> StepReturn:
        # Update curriculum learning
        self._update_curriculum()
        
        if self._thrust_action_term_key:
            thrust_action_term: ThrustAction = self.action_manager._terms[
                self._thrust_action_term_key  # type: ignore
            ]
            remaining_fuel = (
                thrust_action_term.remaining_fuel
                / thrust_action_term.cfg.fuel_capacity
            ).unsqueeze(-1)
        else:
            remaining_fuel = None

        return _compute_step_return(
            episode_length=self.episode_length_buf,
            max_episode_length=self.max_episode_length,
            truncate_episodes=self.cfg.truncate_episodes,
            act_current=self.action_manager.action,
            act_previous=self.action_manager.prev_action,
            tf_pos_robot=self._robot.data.root_pos_w,
            tf_quat_robot=self._robot.data.root_quat_w,
            vel_lin_robot=self._robot.data.root_lin_vel_w,
            vel_ang_robot=self._robot.data.root_ang_vel_w, # Added vel_ang_robot
            # center and params
            center_pos=self._objs.data.object_com_pos_w[:, 0, :],
            radius=torch.tensor(self._radius, device=self.device),
            omega=torch.tensor(self._omega, device=self.device),
            remaining_fuel=remaining_fuel,
        )


@torch.jit.script
def _compute_step_return(
    *,
    episode_length: torch.Tensor,
    max_episode_length: int,
    truncate_episodes: bool,
    act_current: torch.Tensor,
    act_previous: torch.Tensor,
    tf_pos_robot: torch.Tensor,
    tf_quat_robot: torch.Tensor,
    vel_lin_robot: torch.Tensor,
    vel_ang_robot: torch.Tensor, # Added vel_ang_robot
    center_pos: torch.Tensor,
    radius: torch.Tensor,
    omega: torch.Tensor,
    remaining_fuel: torch.Tensor | None,
) -> StepReturn:
    num_envs = episode_length.size(0)
    dtype = episode_length.dtype
    device = episode_length.device

    tf_rot6d_robot = rotmat_to_rot6d(matrix_from_quat(tf_quat_robot))

    rel = tf_pos_robot - center_pos
    rel_xy = rel[:, :2]
    r = torch.norm(rel_xy, dim=-1) + 1e-9
    r_hat = rel_xy / r.unsqueeze(-1)
    t_hat = torch.stack((-r_hat[:, 1], r_hat[:, 0]), dim=-1)

    # Tangential speed target
    v_t = torch.sum(vel_lin_robot[:, :2] * t_hat, dim=-1)
    radial_speed = torch.sum(vel_lin_robot[:, :2] * r_hat, dim=-1)
    radial_err = r - radius

    remaining_fuel = (
        remaining_fuel
        if remaining_fuel is not None
        else torch.ones((num_envs, 1), dtype=dtype, device=device)
    )

    # Rewards
    w_action_rate = -0.01  # Reduced from -0.05 to allow more action exploration
    rew_action_rate = w_action_rate * torch.sum(
        torch.square(act_current - act_previous), dim=1
    )

    w_fuel = -0.5  # Reduced from -2.0 to be less restrictive
    rew_fuel = w_fuel * torch.square(1.0 - remaining_fuel.squeeze(-1))

    w_rad = -1.0  # Reduced from -2.0 for more gradual learning
    rew_rad = w_rad * torch.square(radial_err)

    w_tangential = -0.5  # Reduced from -1.0 for more gradual learning
    rew_tangential = w_tangential * torch.square(v_t - (radius * omega))

    w_rad_speed = -0.1  # Reduced from -0.25 for more gradual learning
    rew_rad_speed = w_rad_speed * torch.square(radial_speed)

    # Add attitude stability reward (new)
    w_attitude = -0.2
    # Get angular velocity for attitude stability
    ang_vel_magnitude = torch.norm(vel_ang_robot, dim=-1)
    rew_attitude = w_attitude * torch.square(ang_vel_magnitude)

    reward_total = rew_action_rate + rew_fuel + rew_rad + rew_tangential + rew_rad_speed + rew_attitude

    termination = torch.zeros(num_envs, dtype=torch.bool, device=device)
    truncation = (
        episode_length >= max_episode_length
        if truncate_episodes
        else torch.zeros(num_envs, dtype=torch.bool, device=device)
    )

    return StepReturn(
        observation={
            "state": {
                "tf_rot6d_robot": tf_rot6d_robot,
                "vel_lin_robot": vel_lin_robot,
                "vel_ang_robot": vel_ang_robot,  # Added angular velocity
                "rel_xy": rel_xy,
                "radial_err": radial_err.unsqueeze(-1),
                "v_t": v_t.unsqueeze(-1),
                "v_r": radial_speed.unsqueeze(-1),
            }
        },
        reward={"total": reward_total},
        termination=termination,
        truncation=truncation,
    )


