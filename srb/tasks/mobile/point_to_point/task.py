"""
Point-to-Point Navigation Task for Spacecraft

A simple 3DOF navigation task where the spacecraft must move from a start point
to an end point in a straight line and stop precisely at the target.
"""

from dataclasses import MISSING
from typing import Sequence, Tuple

import torch
import numpy as np

from srb._typing.step_return import StepReturn
from srb.core.action import ThrustAction
from srb.core.asset import RigidObjectCollection, RigidObjectCollectionCfg, RigidObjectCfg
from srb.core.env.mobile import OrbitalEnv, OrbitalEnvCfg, OrbitalEventCfg, OrbitalSceneCfg
from srb.core.manager import EventTermCfg
from srb.core.sim import PreviewSurfaceCfg, SphereCfg, UsdFileCfg
from srb.utils.cfg import configclass
from srb.utils.math import matrix_from_quat, rotmat_to_rot6d
from srb import assets


@configclass
class SceneCfg(OrbitalSceneCfg):
    env_spacing: float = 25.0  # Same as orbital_evasion
    
    ## Assets
    objs: RigidObjectCollectionCfg = RigidObjectCollectionCfg(
        rigid_objects=MISSING,  # type: ignore
    )


@configclass
class EventCfg(OrbitalEventCfg):
    # no randomization at reset
    randomize_robot_state: EventTermCfg = None


@configclass
class TaskCfg(OrbitalEnvCfg):
    ## Scene
    scene: SceneCfg = SceneCfg()
    
    ## Robot - Use LabSc spacecraft (14-thruster configuration)
    robot = assets.LabSc()
    
    ## Events
    events: EventCfg = EventCfg()
    
    ## Time
    episode_length_s: float = 30.0  # Shorter episodes for point-to-point
    is_finite_horizon: bool = True
    env_rate: float = 1.0 / 200.0  # Same as orbital_evasion (200 Hz)
    
    ## Navigation parameters
    start_distance_m: float = 75.0  # Distance from asteroid to start (same as orbital_evasion)
    target_distance_m: float = 25.0  # Distance from asteroid to target
    
    ## Reward weights
    w_position_error: float = -1.0      # Penalty for distance from target
    w_velocity: float = -0.1            # Penalty for high velocity
    w_action_rate: float = -0.01        # Penalty for rapid action changes
    w_fuel: float = -0.1                # Penalty for fuel consumption
    w_success: float = 10.0             # Bonus for reaching target
    w_attitude: float = -0.2            # Penalty for attitude instability
    w_movement_incentive: float = 0.5   # Bonus for moving toward target

    def __post_init__(self):
        # Completely disable robot randomization - we'll set initial state directly
        self.events.randomize_robot_state = None
        
        # Call the parent's __post_init__ which will set up the robot
        super().__post_init__()
        
        # Ensure at least one rigid object exists for the collection to initialize
        # Use a unique prim path to avoid duplicates
        self.scene.objs.rigid_objects = {
            "apophis": RigidObjectCfg(
                prim_path=f"{{ENV_REGEX_NS}}/apophis_unique",
                spawn=UsdFileCfg(
                    usd_path="/root/ws/srb/assets/object/apophis_accurate.usdz",
                    scale=(1.0, 1.0, 1.0),
                ),
                init_state=RigidObjectCfg.InitialStateCfg(
                    pos=(-100.0, 0.0, 0.0),  # Same as orbital_evasion
                    rot=(1.0, 0.0, 0.0, 0.0),
                ),
            )
        }
        
        # refresh any procedural assets (safe even with empty object list)
        self._update_procedural_assets()


class Task(OrbitalEnv):
    """Point-to-point navigation task implementation."""
    
    cfg: TaskCfg
    
    def __init__(self, cfg: TaskCfg, **kwargs):
        super().__init__(cfg, **kwargs)
        self._objs: RigidObjectCollection = self.scene["objs"]
        
        # Navigation parameters
        self._start_distance = cfg.start_distance_m
        self._target_distance = cfg.target_distance_m
        
        # Asteroid center (same as orbital_evasion)
        self._asteroid_center = torch.tensor([-100.0, 0.0, 0.0], device=self.device)
        
        # Success threshold
        self._success_threshold = 2.0  # meters
        
        # Track if target reached
        self._target_reached = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
        
        # Get thrust action term for fuel tracking
        self._thrust_action_term_key = None
        for key, term in self.action_manager._terms.items():
            if isinstance(term, ThrustAction):
                self._thrust_action_term_key = key
                break
        
        # DEBUG: Print action space and thrust configuration
        print(f"\n=== ACTION SPACE DEBUG ===")
        print(f"Action space shape: {self.action_space.shape}")
        print(f"Action space low: {self.action_space.low}")
        print(f"Action space high: {self.action_space.high}")
        print(f"Number of action terms: {len(self.action_manager._terms)}")
        
        if self._thrust_action_term_key:
            thrust_term = self.action_manager._terms[self._thrust_action_term_key]
            print(f"Thrust action term key: {self._thrust_action_term_key}")
            print(f"Thrust action term type: {type(thrust_term)}")
            if hasattr(thrust_term, '_thruster_power'):
                print(f"Thruster powers: {thrust_term._thruster_power}")
            if hasattr(thrust_term, '_thruster_direction'):
                print(f"Thruster directions shape: {thrust_term._thruster_direction.shape}")
        else:
            print("❌ NO THRUST ACTION TERM FOUND!")
        print("=" * 50)
        
        # Initialize positions immediately
        self._initialize_positions()

    def _initialize_positions(self):
        """Initialize start and target positions for all environments."""
        # Generate start and target positions for all environments
        start_angles = torch.rand(self.num_envs, device=self.device) * 2 * torch.pi
        target_angles = torch.rand(self.num_envs, device=self.device) * 2 * torch.pi
        
        # Start positions at start_distance from asteroid center
        start_x = self._asteroid_center[0] + self._start_distance * torch.cos(start_angles)
        start_y = self._asteroid_center[1] + self._start_distance * torch.sin(start_angles)
        start_z = self._asteroid_center[2] * torch.ones_like(start_angles)  # Same Z as asteroid
        self._start_pos = torch.stack([start_x, start_y, start_z], dim=-1)
        
        # Target positions at target_distance from asteroid center
        target_x = self._asteroid_center[0] + self._target_distance * torch.cos(target_angles)
        target_y = self._asteroid_center[1] + self._target_distance * torch.sin(target_angles)
        target_z = self._asteroid_center[2] * torch.ones_like(target_angles)  # Same Z as asteroid
        self._target_pos = torch.stack([target_x, target_y, target_z], dim=-1)

    def _reset_idx(self, env_ids: Sequence[int]):
        """Reset specific environments."""
        super()._reset_idx(env_ids)
        
        # Reset target reached flag
        self._target_reached[env_ids] = False
        
        # Generate new start and target positions for specific environments
        start_angles = torch.rand(len(env_ids), device=self.device) * 2 * torch.pi
        target_angles = torch.rand(len(env_ids), device=self.device) * 2 * torch.pi
        
        # Start positions at start_distance from asteroid center
        start_x = self._asteroid_center[0] + self._start_distance * torch.cos(start_angles)
        start_y = self._asteroid_center[1] + self._start_distance * torch.sin(start_angles)
        start_z = self._asteroid_center[2] * torch.ones_like(start_angles)  # Same Z as asteroid
        new_start_pos = torch.stack([start_x, start_y, start_z], dim=-1)
        
        # Target positions at target_distance from asteroid center
        target_x = self._asteroid_center[0] + self._target_distance * torch.cos(target_angles)
        target_y = self._asteroid_center[1] + self._target_distance * torch.sin(target_angles)
        target_z = self._asteroid_center[2] * torch.ones_like(target_angles)  # Same Z as asteroid
        new_target_pos = torch.stack([target_x, target_y, target_z], dim=-1)
        
        # Update positions for specific environments
        self._start_pos[env_ids] = new_start_pos
        self._target_pos[env_ids] = new_target_pos
        
        # Set spacecraft to new start positions (same pattern as orbital_evasion)
        n = len(env_ids)
        self._robot.data.root_pos_w[env_ids] = new_start_pos
        
        # Set orientation (same as orbital_evasion)
        qz_90 = torch.tensor([0.70710678, 0.0, 0.0, 0.70710678], device=self.device)
        self._robot.data.root_quat_w[env_ids] = qz_90.unsqueeze(0).repeat(n, 1)
        
        # Set velocities to zero
        self._robot.data.root_lin_vel_w[env_ids] = 0.0
        self._robot.data.root_ang_vel_w[env_ids] = 0.0
        
        # Write to simulation (same pattern as orbital_evasion)
        static_pose = torch.cat([new_start_pos, qz_90.unsqueeze(0).repeat(n, 1)], dim=-1)
        static_velocity = torch.zeros(n, 6, device=self.device, dtype=torch.float32)
        self._robot.write_root_pose_to_sim(static_pose, env_ids=env_ids)
        self._robot.write_root_velocity_to_sim(static_velocity, env_ids=env_ids)

    def extract_step_return(self) -> StepReturn:
        """Compute the step return for the current state."""
        
        # DEBUG: Print current actions and spacecraft state
        if hasattr(self, '_debug_counter'):
            self._debug_counter += 1
        else:
            self._debug_counter = 0
            
        if self._debug_counter % 100 == 0:  # Print every 100 steps
            print(f"\n=== DEBUG STEP {self._debug_counter} ===")
            print(f"Current actions: {self.action_manager.action[0].cpu().numpy()}")
            print(f"Spacecraft position: {self._robot.data.root_pos_w[0].cpu().numpy()}")
            print(f"Spacecraft velocity: {self._robot.data.root_lin_vel_w[0].cpu().numpy()}")
            print(f"Target position: {self._target_pos[0].cpu().numpy()}")
            print(f"Distance to target: {torch.norm(self._robot.data.root_pos_w[0] - self._target_pos[0]).cpu().numpy():.2f}")
            
            # Check if thrust action term exists and has fuel
            if self._thrust_action_term_key:
                thrust_term = self.action_manager._terms[self._thrust_action_term_key]
                print(f"Thrust action term: {type(thrust_term)}")
                if hasattr(thrust_term, 'remaining_fuel'):
                    print(f"Remaining fuel: {thrust_term.remaining_fuel[0].cpu().numpy()}")
            print("=" * 50)
        
        if self._thrust_action_term_key:
            thrust_action_term: ThrustAction = self.action_manager._terms[
                self._thrust_action_term_key
            ]
            remaining_fuel = thrust_action_term.remaining_fuel
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
            vel_ang_robot=self._robot.data.root_ang_vel_w,
            target_pos=self._target_pos,  # Now a tensor of shape [num_envs, 3]
            success_threshold=self._success_threshold,
            remaining_fuel=remaining_fuel,
            target_reached=self._target_reached,
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
    vel_ang_robot: torch.Tensor,
    target_pos: torch.Tensor,  # Shape: [num_envs, 3]
    success_threshold: float,
    remaining_fuel: torch.Tensor | None,
    target_reached: torch.Tensor,
) -> StepReturn:
    num_envs = episode_length.size(0)
    dtype = episode_length.dtype
    device = episode_length.device

    # Convert quaternion to rotation matrix
    tf_rot6d_robot = rotmat_to_rot6d(matrix_from_quat(tf_quat_robot))

    # Calculate distance to target (target_pos is already [num_envs, 3])
    distance_to_target = torch.norm(tf_pos_robot - target_pos, dim=-1)
    
    # Check if target reached
    target_reached = distance_to_target < success_threshold
    
    # Calculate velocity magnitude
    velocity_magnitude = torch.norm(vel_lin_robot, dim=-1)
    
    # Calculate angular velocity magnitude
    ang_vel_magnitude = torch.norm(vel_ang_robot, dim=-1)

    # Handle fuel
    remaining_fuel = (
        remaining_fuel
        if remaining_fuel is not None
        else torch.ones((num_envs, 1), dtype=dtype, device=device)
    )

    # Rewards
    w_position_error = -1.0
    rew_position_error = w_position_error * distance_to_target

    w_velocity = -0.1
    rew_velocity = w_velocity * torch.square(velocity_magnitude)

    w_action_rate = -0.01
    rew_action_rate = w_action_rate * torch.sum(
        torch.square(act_current - act_previous), dim=1
    )

    w_fuel = -0.1
    rew_fuel = w_fuel * torch.square(1.0 - remaining_fuel.squeeze(-1))

    w_success = 10.0
    rew_success = w_success * target_reached.float()

    w_attitude = -0.2
    rew_attitude = w_attitude * torch.square(ang_vel_magnitude)

    # Movement incentive: reward for moving toward target
    w_movement_incentive = 0.5
    # Calculate if we're moving toward target (negative dot product means moving toward)
    robot_to_target = target_pos - tf_pos_robot
    robot_to_target_normalized = robot_to_target / (torch.norm(robot_to_target, dim=-1, keepdim=True) + 1e-6)
    movement_toward_target = -torch.sum(vel_lin_robot * robot_to_target_normalized, dim=-1)
    rew_movement_incentive = w_movement_incentive * torch.clamp(movement_toward_target, min=0.0)

    # Total reward
    reward_total = (
        rew_position_error + 
        rew_velocity + 
        rew_action_rate + 
        rew_fuel + 
        rew_success + 
        rew_attitude +
        rew_movement_incentive
    )

    # Termination conditions
    termination = target_reached  # Episode ends when target reached
    
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
                "vel_ang_robot": vel_ang_robot,
                "target_pos": target_pos,  # Already [num_envs, 3]
                "distance_to_target": distance_to_target.unsqueeze(-1),
                "velocity_magnitude": velocity_magnitude.unsqueeze(-1),
                "target_reached": target_reached.unsqueeze(-1),
            }
        },
        reward={"total": reward_total},
        termination=termination,
        truncation=truncation,
    )
