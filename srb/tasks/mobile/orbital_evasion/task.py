from dataclasses import MISSING
from typing import Sequence, Tuple

import torch

from srb._typing.step_return import StepReturn
from srb.core.action import ThrustAction
from srb.core.asset import RigidObjectCollection, RigidObjectCollectionCfg
from srb.core.env.mobile import OrbitalEnv, OrbitalEnvCfg, OrbitalEventCfg, OrbitalSceneCfg
from srb.core.manager import EventTermCfg, SceneEntityCfg
from srb.core.marker import VisualizationMarkers, VisualizationMarkersCfg
from srb.core.mdp import reset_collection_root_state_uniform_poisson_disk_3d
from srb.core.sim import PreviewSurfaceCfg, SphereCfg
from srb.utils.cfg import configclass
from srb.utils.math import deg_to_rad, matrix_from_quat, rotmat_to_rot6d, quat_from_two_vectors

from .asset import select_obstacle
from srb.core.asset import RigidObjectCfg
from srb.core.sim import UsdFileCfg


##############
### Config ###
##############

@configclass
class SceneCfg(OrbitalSceneCfg):
    env_spacing: float = 25.0

    ## Assets
    objs: RigidObjectCollectionCfg = RigidObjectCollectionCfg(
        rigid_objects=MISSING,  # type: ignore
    )


@configclass
class EventCfg(OrbitalEventCfg):
    # no randomization at reset
    randomize_object_state: EventTermCfg = None


@configclass
class TaskCfg(OrbitalEnvCfg):
    ## Scene
    scene: SceneCfg = SceneCfg()
    num_obstacles: int = 1

    ## Events
    events: EventCfg = EventCfg()

    ## Time
    episode_length_s: float = 30.0
    is_finite_horizon: bool = False

    ## Debug motion (meters & rad/step). 0 amp => fully static.
    debug_robot_orbit_amp_m: float = 0.0
    debug_robot_orbit_speed_rad: float = 0.0

    ## Target marker (not used for motion, just visualization)
    tf_pos_target: Tuple[float, float, float] = (0.0, 0.0, -50.0)
    tf_quat_target: Tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)
    target_marker_cfg: VisualizationMarkersCfg = VisualizationMarkersCfg(
        prim_path="/Visuals/target",
        markers={
            "target": SphereCfg(
                radius=0.25,
                visual_material=PreviewSurfaceCfg(emissive_color=(0.2, 0.2, 0.8)),
            )
        },
    )

    def __post_init__(self):
        super().__post_init__()

        # Place the Apophis model 100m back along -X
        self.scene.objs.rigid_objects = {
            "apophis": RigidObjectCfg(
                prim_path=f"{{ENV_REGEX_NS}}/apophis",
                spawn=UsdFileCfg(
                    usd_path="/root/ws/srb/assets/object/apophis_accurate.usdz",
                    scale=(1.0, 1.0, 1.0),
                ),
                init_state=RigidObjectCfg.InitialStateCfg(
                    pos=(-100.0, 0.0, 0.0),
                    rot=(1.0, 0.0, 0.0, 0.0),
                ),
            )
        }

        # refresh any procedural assets
        self._update_procedural_assets()


############
### Task ###
############

class Task(OrbitalEnv):
    cfg: TaskCfg

    def __init__(self, cfg: TaskCfg, **kwargs):
        print("[TASK] Initializing regular Task (not VisualTask)...")
        super().__init__(cfg, **kwargs)
        self._objs: RigidObjectCollection = self.scene["objs"]
        self._orbit_angle = 0.0
        self._orbit_radius = 150.0  # 75 meters away from Apophis
        self._orbit_center = torch.tensor([-100.0, 0.0, 0.0], device=self.device)  # Z fixed to match Apophis
        self._orbit_angular_speed = 0.0005  # radians per step, slower orbit for easier following
        self._debug_counter = 0  # Counter to control debug print frequency
        self._dbg_phase = 0.0  # Debug motion phase
        # Restore target marker and pose attributes
        self._target_marker = VisualizationMarkers(self.cfg.target_marker_cfg)
        self._tf_pos_target = (
            self.scene.env_origins
            + torch.tensor(self.cfg.tf_pos_target, device=self.device)
        ).repeat(self.num_envs, 1)
        self._tf_quat_target = torch.tensor(
            self.cfg.tf_quat_target, device=self.device
        ).repeat(self.num_envs, 1)
        self._target_marker.visualize(self._tf_pos_target, self._tf_quat_target)
        # Print thruster configuration at startup
        if hasattr(self, 'action_manager') and self.action_manager is not None:
            for term_name, term in self.action_manager._terms.items():
                if hasattr(term, '_thruster_direction'):
                    print("\n=== Thruster Directions ===\n", term._thruster_direction)
                    print("=== Thruster Powers ===\n", term._thruster_power)
        # Remove spacecraft marker

    def _reset_idx(self, env_ids: Sequence[int]):
        super()._reset_idx(env_ids)
        n = len(env_ids)
        # Start the spacecraft at the first desired orbit position in XY plane, Z fixed
        theta0 = 0.0
        start_pos = self._orbit_center.clone()
        start_pos[0] += self._orbit_radius * torch.cos(torch.tensor(theta0, device=self.device))
        start_pos[1] += self._orbit_radius * torch.sin(torch.tensor(theta0, device=self.device))
        # start_pos[2] = Z fixed to match Apophis (0.0)
        self._robot.data.root_pos_w[env_ids] = start_pos.unsqueeze(0).repeat(n, 1)
        qz_90 = torch.tensor([0.70710678, 0.0, 0.0, 0.70710678], device=self.device)
        self._robot.data.root_quat_w[env_ids] = qz_90.unsqueeze(0).repeat(n, 1)
        self._robot.data.root_lin_vel_w[env_ids] = 0.0
        self._robot.data.root_ang_vel_w[env_ids] = 0.0
        self._robot.data.root_lin_vel_b[env_ids] = 0.0
        self._robot.data.root_ang_vel_b[env_ids] = 0.0
        static_pose = torch.cat([start_pos, qz_90], dim=-1).unsqueeze(0).repeat(n, 1)
        static_velocity = torch.zeros(n, 6, device=self.device, dtype=torch.float32)
        self._robot.write_root_pose_to_sim(static_pose, env_ids=env_ids)
        self._robot.write_root_velocity_to_sim(static_velocity, env_ids=env_ids)
        if hasattr(self._robot, 'root_physx_view'):
            self._robot.root_physx_view.set_masses(torch.full((n,), 10.0, device=self.device), indices=self._robot._ALL_INDICES)
            self._robot.root_physx_view.apply_forces_and_torques_at_position(
                force_data=torch.zeros((n, 3), device=self.device),
                torque_data=torch.zeros((n, 3), device=self.device),
                position_data=self._robot.root_physx_view.get_coms()[:, :3],
                indices=self._robot._ALL_INDICES,
                is_global=False,
            )
        self._objs.data.object_com_pos_w[env_ids] = torch.tensor(
            [[[-100.0, 0.0, 0.0]]] * n, device=self.device
        )
        self._objs.data.object_com_quat_w[env_ids] = torch.tensor([1.0, 0.0, 0.0, 0.0], device=self.device).unsqueeze(0).repeat(n, 1)
        self._objs.data.object_lin_vel_b[env_ids]   = 0.0
        self._objs.data.object_ang_vel_b[env_ids]   = 0.0

    def extract_step_return(self) -> StepReturn:
        # unchanged reward/obs logic
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
            vel_lin_robot=self._robot.data.root_lin_vel_b,
            vel_ang_robot=self._robot.data.root_ang_vel_b,
            tf_pos_objs=self._objs.data.object_com_pos_w,
            tf_pos_target=self._tf_pos_target,
            imu_lin_acc=self._imu_robot.data.lin_acc_b,
            imu_ang_vel=self._imu_robot.data.ang_vel_b,
            remaining_fuel=remaining_fuel,
        )


    # NEW – run *before* the physics integrator each frame
    def _pre_physics_step(self, action: torch.Tensor | None):
        self._orbit_angle += self._orbit_angular_speed
        desired_pos = self._orbit_center.clone()
        desired_pos[0] += self._orbit_radius * torch.cos(torch.tensor(self._orbit_angle, device=self.device))
        desired_pos[1] += self._orbit_radius * torch.sin(torch.tensor(self._orbit_angle, device=self.device))
        # desired_pos[2] = Z fixed to match Apophis (0.0)
        current_pos = self._robot.data.root_pos_w[0]
        current_vel = self._robot.data.root_lin_vel_w[0]
        current_quat = self._robot.data.root_quat_w[0]
        current_ang_vel = self._robot.data.root_ang_vel_w[0]
        
        # Much more aggressive gains for orbital motion
        kp_pos = 0.5  # Increased significantly
        kd_pos = 0.8  # Increased significantly
        pos_error = desired_pos - current_pos
        vel_error = -current_vel
        desired_acc = kp_pos * pos_error + kd_pos * vel_error
        # CRITICAL: Clamp Z control - spacecraft should NEVER move in Z axis
        desired_acc[2] = 0.0
        
        # XY-ONLY APPROACH: Use XY thrusters for XY motion, no Z movement
        xy_magnitude = torch.norm(desired_acc[:2])
        
        if xy_magnitude > 0.01:
            # Calculate the angle to point thrusters in the desired direction
            desired_direction = desired_acc[:2] / xy_magnitude
            
            # Calculate the angle to rotate around Z-axis to point in desired direction
            angle_to_rotate = torch.atan2(desired_direction[1], desired_direction[0])
            
            # Create desired angular acceleration for Z rotation (yaw)
            kp_yaw = 2.0  # Strong gain for Z rotation
            kd_yaw = 0.5  # Damping for Z rotation
            
            # Calculate current yaw angle from quaternion
            current_yaw = 2 * torch.atan2(current_quat[3], current_quat[0])
            
            # Calculate yaw error
            yaw_error = angle_to_rotate - current_yaw
            
            # Normalize yaw error to [-pi, pi]
            while yaw_error > torch.pi:
                yaw_error -= 2 * torch.pi
            while yaw_error < -torch.pi:
                yaw_error += 2 * torch.pi
            
            # Set desired angular acceleration for Z rotation
            desired_ang_acc = torch.zeros(3, device=self.device)
            desired_ang_acc[2] = kp_yaw * yaw_error + kd_yaw * (-current_ang_vel[2])
        else:
            # If no X/Y movement needed, just maintain current orientation
            desired_ang_acc = torch.zeros(3, device=self.device)
        
        # **STATIC SPACECRAFT**: Set all actions to zero to keep spacecraft completely static
        if hasattr(self, 'action_manager') and self.action_manager is not None:
            action_dim = self.action_manager.action.shape[1]
            action_out = torch.zeros((self.num_envs, action_dim), device=self.device)
            
            # Set all actions to zero - no movement at all
            self.action_manager.action.copy_(action_out)
            self.action_manager.process_action(self.action_manager.action)

        # **DEBUG MOTION**: Apply tiny orbital wiggle if requested
        amp = getattr(self.cfg, "debug_robot_orbit_amp_m", 0.0)
        spd = getattr(self.cfg, "debug_robot_orbit_speed_rad", 0.0)

        if amp > 0.0 and spd != 0.0:
            self._dbg_phase += spd
            dx = amp * torch.cos(torch.tensor(self._dbg_phase, device=self.device))
            dy = amp * torch.sin(torch.tensor(self._dbg_phase, device=self.device))
            # Use current robot position as base, add tiny orbital motion
            base_pos = self._robot.data.root_pos_w[0].clone()
            pos_robot = base_pos + torch.tensor([dx, dy, 0.0], device=self.device)
            # Apply the debug motion to the robot
            self._robot.data.root_pos_w[0] = pos_robot
        
        # **DISABLED ORBIT DEBUG OUTPUT** - Commented out to reduce noise
        # self._debug_counter += 1
        # if self._debug_counter % 50 == 0:
        #     print(f"[ORBIT] Step {self._debug_counter}: angle={self._orbit_angle:.3f}, pos_error_xy={torch.norm(pos_error[:2]):.2f}, xy_mag={xy_magnitude:.3f}")
        #     print(f"[ORBIT] Desired: {desired_pos[:2].cpu().numpy()}, Current: {current_pos[:2].cpu().numpy()}")
        #     print(f"[ORBIT] Z-pos: {current_pos[2].cpu().numpy():.3f} (should stay ~0)")
        #     print(f"[ORBIT] XY Action: {action_out[0, :5].cpu().numpy()}")
        #     print("---")

        # Freeze velocities every tick so PhysX can't drift us
        self._robot.data.root_lin_vel_b[:] = 0.0
        self._robot.data.root_ang_vel_b[:] = 0.0
        self._objs.data.object_lin_vel_b[:] = 0.0
        self._objs.data.object_ang_vel_b[:] = 0.0

    def _post_physics_step(self):
        pass



    def _step_env(self, env_ids: Sequence[int]):
        # **DO NOT** move the spacecraft or asteroid at all
        # just step through any internal physics/sensors if needed
        pass


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
    tf_pos_objs: torch.Tensor,
    tf_pos_target: torch.Tensor,
    imu_lin_acc: torch.Tensor,
    imu_ang_vel: torch.Tensor,
    remaining_fuel: torch.Tensor | None,
) -> StepReturn:
    num_envs = episode_length.size(0)
    num_objs = tf_pos_objs.size(1)
    dtype = episode_length.dtype
    device = episode_length.device

    # --- States ---
    tf_rotmat_robot = matrix_from_quat(tf_quat_robot)
    tf_rot6d_robot = rotmat_to_rot6d(tf_rotmat_robot)

    pos_robot_to_objs = (
        tf_pos_robot.unsqueeze(1).repeat(1, num_objs, 1) - tf_pos_objs
    )
    tf_pos_robot_to_nearest_obj = pos_robot_to_objs[
        torch.arange(pos_robot_to_objs.size(0)),
        torch.argmin(torch.norm(pos_robot_to_objs, dim=-1), dim=1),
    ]
    tf_pos_robot_to_target = tf_pos_target - tf_pos_robot

    remaining_fuel = (
        remaining_fuel
        if remaining_fuel is not None
        else torch.ones((num_envs, 1), dtype=dtype, device=device)
    )

    # --- Rewards ---
    W_AR = -0.025
    penalty_action_rate = W_AR * torch.sum(
        torch.square(act_current - act_previous), dim=1
    )

    W_FC = -2.0
    penalty_fuel_consumption = W_FC * torch.square(
        1.0 - (remaining_fuel.squeeze(-1) if remaining_fuel is not None else torch.ones((num_envs,), dtype=dtype, device=device))
    )

    W_AV = -0.25
    MAX_AV = -5.0
    penalty_angular_velocity = torch.clamp_min(
        W_AV * torch.sum(torch.square(vel_ang_robot), dim=1), min=MAX_AV
    )

    W_DR2O = 2.0
    reward_distance_robot_to_nearest_obj = W_DR2O * torch.norm(
        tf_pos_robot_to_nearest_obj, dim=-1
    )

    W_DRT = -32.0
    penalty_distance_robot_to_target = W_DRT * torch.norm(
        tf_pos_robot_to_target, dim=-1
    )

    # --- Terminations ---
    termination = torch.zeros(num_envs, dtype=torch.bool, device=device)
    truncation = (
        episode_length >= max_episode_length
        if truncate_episodes
        else torch.zeros(num_envs, dtype=torch.bool, device=device)
    )

    return StepReturn(
        {
            "state": {
                "tf_rot6d_robot": tf_rot6d_robot,
                "vel_lin_robot": vel_lin_robot,
                "vel_ang_robot": vel_ang_robot,
                "tf_pos_robot_to_target": tf_pos_robot_to_target,
                "pos_robot_to_objs": pos_robot_to_objs,
            },
            "proprio": {
                "imu_lin_acc": imu_lin_acc,
                "imu_ang_vel": imu_ang_vel,
                "remaining_fuel": remaining_fuel,
            },
        },
        reward={
            "total": (
                penalty_action_rate
                + penalty_fuel_consumption
                + penalty_angular_velocity
                + reward_distance_robot_to_nearest_obj
                + penalty_distance_robot_to_target
            )
        },
        termination=termination,
        truncation=truncation,
    )
