from typing import TYPE_CHECKING, Sequence, Tuple, Type

import torch
from pydantic import BaseModel

from srb.core.manager import ActionTerm, ActionTermCfg
from srb.core.marker import ARROW_CFG, VisualizationMarkers
from srb.core.sim import PreviewSurfaceCfg
from srb.utils.cfg import configclass
from srb.utils.math import (
    combine_frame_transforms,
    matrix_from_euler,
    normalize,
    quat_apply,
    quat_from_matrix,
)

if TYPE_CHECKING:
    from srb._typing import AnyEnv
    from srb.core.asset import RigidObject


class ThrustAction(ActionTerm):
    cfg: "ThrustActionCfg"
    _env: "AnyEnv"
    _asset: "RigidObject"

    def __init__(self, cfg: "ThrustActionCfg", env: "AnyEnv"):
        super().__init__(
            cfg,
            env,  # type: ignore
        )

        ## Pre-process thrusters
        thruster_offset = []
        thruster_direction = []
        thruster_power = []
        thruster_gimbal_limits = []
        for thruster_cfg in cfg.thrusters:
            thruster_offset.append(thruster_cfg.offset)
            direction_norm = (
                thruster_cfg.direction[0] ** 2
                + thruster_cfg.direction[1] ** 2
                + thruster_cfg.direction[2] ** 2
            ) ** 0.5
            assert direction_norm > 0, (
                "Thruster direction must have a non-zero magnitude"
            )
            direction = (
                thruster_cfg.direction[0] / direction_norm,
                thruster_cfg.direction[1] / direction_norm,
                thruster_cfg.direction[2] / direction_norm,
            )
            thruster_direction.append(direction)
            thruster_gimbal_limits.append(thruster_cfg.gimbal_limits or (0.0, 0.0))
            thruster_power.append(thruster_cfg.power)
        self._thruster_offset = torch.tensor(thruster_offset, device=env.device)
        self._thruster_direction = torch.tensor(thruster_direction, device=env.device)
        self._thruster_power = torch.tensor(thruster_power, device=env.device)
        self._thruster_gimbal_limits = torch.tensor(
            thruster_gimbal_limits, device=env.device
        )

        ## Set up action indices
        self._num_thrusters = len(cfg.thrusters)
        self._action_indices_thrust = torch.arange(
            self._num_thrusters, device=env.device
        )
        self._num_thrusters_with_gimbal = sum(1 for lim in thruster_gimbal_limits if lim != (0.0, 0.0))
        action_indices_gimbal = []
        next_idx = self._num_thrusters
        for lim in thruster_gimbal_limits:
            if lim == (0.0, 0.0):
                action_indices_gimbal.append((-1, -1))
            else:
                action_indices_gimbal.append((next_idx, next_idx + 1))
                next_idx += 2
        self._action_indices_gimbal = torch.tensor(
            action_indices_gimbal, device=env.device
        )

        ## Initialize fuel & mass
        self._remaining_fuel = cfg.fuel_capacity * torch.ones(
            env.num_envs, device=env.device
        )
        self._dry_masses = self._asset.root_physx_view.get_masses().clone()

        ## Initialize action tensors
        self._raw_actions = torch.zeros((env.num_envs, self.action_dim), device=env.device)
        self._processed_actions = torch.zeros((env.num_envs, self.action_dim), device=env.device)

        ## Set up visualization markers
        if self.cfg.debug_vis:
            self._setup_visualization_markers()

    @property
    def action_dim(self) -> int:
        # If we are in world-XY control mode, first two dims are Fx,Fy commands.
        base_dim = 2 if self.cfg.control_xy else self._num_thrusters
        return base_dim + 2 * self._num_thrusters_with_gimbal

    @property
    def raw_actions(self) -> torch.Tensor:
        return self._raw_actions

    @property
    def processed_actions(self) -> torch.Tensor:
        return self._processed_actions

    @property
    def remaining_fuel(self) -> torch.Tensor:
        return self._remaining_fuel

    def process_actions(self, actions):
        self._raw_actions = actions
        self._processed_actions = actions.clone()
        if self.cfg.control_xy:
            # First two dims are desired Fx,Fy in range [-1,1]
            self._processed_actions[:, 0:2] = torch.clamp(
                self._processed_actions[:, 0:2], -1.0, 1.0
            )
        else:
            # Per-thruster magnitude commands in [0,1]
            self._processed_actions[:, self._action_indices_thrust] = torch.clamp(
                self._processed_actions[:, self._action_indices_thrust], 0.0, 1.0
            )

    def apply_actions(self):
        # Debug: Print received actions
        if hasattr(self, '_processed_actions') and self._processed_actions is not None:
            action_sum = self._processed_actions.sum().item()
            if action_sum > 0.01:  # Only print when there's significant action
                print(f"[THRUST_DEBUG] Applying actions: {self._processed_actions[0]}")
                print(f"[THRUST_DEBUG] Action sum: {action_sum:.3f}")
        
        # ===== 1) Gimbal in body frame (as you already do) =====
        thruster_directions_b = (
            self._thruster_direction.unsqueeze(0).expand(self.num_envs, -1, -1).clone()
        )
        for i in range(self._num_thrusters):
            gimbal_i = self._action_indices_gimbal[i]
            if gimbal_i[0] < 0:
                continue
            lim = self._thruster_gimbal_limits[i]
            gimbal_x = lim[0] * self._processed_actions[:, gimbal_i[0]].clamp(-1.0, 1.0)
            gimbal_y = lim[1] * self._processed_actions[:, gimbal_i[1]].clamp(-1.0, 1.0)
            euler_angles = torch.zeros((self.num_envs, 3), device=self.device)
            euler_angles[:, 0] = gimbal_x
            euler_angles[:, 1] = gimbal_y
            R_b = matrix_from_euler(euler_angles, convention="XYZ")
            nominal_dir_b = self._thruster_direction[i].unsqueeze(0).expand(self.num_envs, -1)
            thruster_directions_b[:, i, :] = torch.bmm(R_b, nominal_dir_b.unsqueeze(2)).squeeze(2)

        # ===== 2) Transform to WORLD frame (compute early for allocator too) =====
        asset_pos_w  = self._asset.data.root_pos_w                 # [N,3]
        asset_quat_w = self._asset.data.root_quat_w                # [N,4]
        com_pos_w    = self._asset.root_physx_view.get_coms()[:, :3]  # [N,3]

        # ---- yaw tracking (store initial yaw) ----
        x_body_vec = torch.tensor([1.0, 0.0, 0.0], device=self.device).expand(self.num_envs, -1)
        x_world_vec = quat_apply(asset_quat_w, x_body_vec)
        current_yaw = torch.atan2(x_world_vec[:, 1], x_world_vec[:, 0])  # [N]
        if not hasattr(self, "_yaw_target"):
            self._yaw_target = current_yaw.detach().clone()

        thruster_offsets_b = self._thruster_offset.unsqueeze(0).expand(self.num_envs, -1, -1)  # [N,T,3]

        # thruster positions in world: p_w = p0_w + R_wb * offset_b
        thruster_pos_w = asset_pos_w.unsqueeze(1) + quat_apply(
            asset_quat_w.unsqueeze(1), thruster_offsets_b
        )  # [N,T,3]

        # thruster directions in world: d_w = R_wb * d_b
        thruster_directions_w = quat_apply(
            asset_quat_w.unsqueeze(1), thruster_directions_b
        )  # [N,T,3]

        # === Project directions to XY plane in WORLD (keep true magnitude) ===
        dir_xy = thruster_directions_w.clone()
        dir_xy[..., 2] = 0.0  # zero Z component

        # ------------------------------------------------------------------
        #  Force on the spacecraft is **opposite** to exhaust flow
        # ------------------------------------------------------------------
        dir_xy = -dir_xy            # <── THIS LINE IS THE KEY

        # ===== 2.5) Magnitudes (after world transforms available) =====
        if self.cfg.control_xy:
            # Desired planar world force vector in Newtons
            FxFy_cmd = self._processed_actions[:, 0:2] * self.cfg.max_planar_force  # [N,2]
            desired_wrench = torch.zeros((self.num_envs, 3, 1), device=self.device)
            desired_wrench[:, 0, 0] = FxFy_cmd[:, 0]
            desired_wrench[:, 1, 0] = FxFy_cmd[:, 1]
            desired_wrench[:, 2, 0] = 0.0

            # Use unnormalized dir_xy for correct in-plane authority
            Fx_cols = (self.cfg.scale * self._thruster_power).unsqueeze(0) * dir_xy[..., 0]
            Fy_cols = (self.cfg.scale * self._thruster_power).unsqueeze(0) * dir_xy[..., 1]
            r_x = (thruster_pos_w - com_pos_w.unsqueeze(1))[..., 0]
            r_y = (thruster_pos_w - com_pos_w.unsqueeze(1))[..., 1]
            Tau_cols = r_x * Fy_cols - r_y * Fx_cols
            A = torch.stack([Fx_cols, Fy_cols, Tau_cols], dim=1)  # [N,3,T]
            At = A.transpose(1, 2)

            # Projected gradient descent with box constraints 0<=u<=1
            u = torch.zeros((self.num_envs, self._num_thrusters, 1), device=self.device)
            lam = self.cfg.allocator_lambda
            lr = self.cfg.allocator_lr
            for _ in range(self.cfg.allocator_steps):
                resid = torch.bmm(A, u) - desired_wrench
                grad = torch.bmm(At, resid) + lam * u
                u = torch.clamp(u - lr * grad, 0.0, 1.0)
            thrust_magnitudes = u.squeeze(-1) * self._thruster_power.unsqueeze(0)
        else:
            thrust_actions = self._processed_actions[:, self._action_indices_thrust]
            thrust_magnitudes = thrust_actions * self._thruster_power.unsqueeze(0)

        thrust_magnitudes *= (self._remaining_fuel.unsqueeze(-1) > 0.0)

        # ===== 4) Forces & torques in WORLD (PLANAR 3-DOF APPROACH) =====
        # 'direction' now means **force direction on the spacecraft** (intuitive)
        thruster_forces_w = self.cfg.scale * thrust_magnitudes.unsqueeze(-1) * dir_xy  # [N,T,3]
        
        # 1) PLANARIZE moment arms - kill lever-arm in Z to eliminate roll/pitch torques
        r_w = thruster_pos_w - com_pos_w.unsqueeze(1)   # [N,T,3]
        r_w[..., 2] = 0.0                               # ✶ kill lever-arm in Z
        
        # 2) Compute torques with planarized arms (only yaw component will remain)
        thruster_torques_w = torch.cross(r_w.float(), thruster_forces_w.float(), dim=2)  # [N,T,3]
        
        # 3) Aggregate forces and torques
        total_forces_w = thruster_forces_w.sum(dim=1)  # [N,3]
        total_forces_w[..., 2] = 0.0                   # enforce planar force (sanity check)
        
        total_torques_w = torch.zeros_like(total_forces_w)
        total_torques_w[..., 2] = thruster_torques_w[..., 2].sum(dim=1)  # yaw only
        
        # ===== Apply minimal attitude control for 3-DOF planar motion =====
        if self.cfg.lock_roll_pitch:
            # Light damping on roll/pitch rates (should be minimal now that lever arms are planarized)
            kd_rp = 2.0  # Reduced since we eliminated the main source of roll/pitch disturbance
            rp_rates = self._asset.data.root_ang_vel_w[..., 0:2]
            total_torques_w[..., 0:2] = -kd_rp * rp_rates
        if self.cfg.lock_yaw:
            # PD yaw hold to initial yaw (can still be used for yaw control)
            kp_yaw = 10.0
            kd_yaw = 5.0
            yaw_err = torch.atan2(torch.sin(current_yaw - self._yaw_target),
                                   torch.cos(current_yaw - self._yaw_target))
            yaw_rate = self._asset.data.root_ang_vel_w[..., 2]
            total_torques_w[..., 2] += -kp_yaw * yaw_err - kd_yaw * yaw_rate  # Add to thruster yaw torques

        # DEBUG: Print 3-DOF planar motion forces and torques
        if total_forces_w.abs().sum() > 0.1:  # Only when forces are applied
            net_fz = total_forces_w[0][2].item()
            net_tau_xy = total_torques_w[0][:2].abs().sum().item()
            net_tau_z = total_torques_w[0][2].item()
            
            print(f"[3DOF_DEBUG] Aggregated wrench at CoM:")
            print(f"  Force: ({total_forces_w[0][0]:.1f}, {total_forces_w[0][1]:.1f}, {total_forces_w[0][2]:.1f}) N")
            print(f"  Torque: ({total_torques_w[0][0]:.1f}, {total_torques_w[0][1]:.1f}, {total_torques_w[0][2]:.1f}) N⋅m")
            print(f"  ✅ Z-force: {net_fz:.1f}N (should be ~0)")
            print(f"  ✅ Roll/pitch torque: {net_tau_xy:.1f} N⋅m (should be ~0)")
            print(f"  ✅ Yaw torque: {net_tau_z:.1f} N⋅m (thruster yaw)")
            
            if abs(net_fz) > 10.0:
                print(f"  🚨 WARNING: Large Z force will cause vertical motion!")
            if net_tau_xy > 5.0:
                print(f"  🚨 WARNING: Large roll/pitch torque detected!")
            
            # Track actual dynamics effect (world-frame dv)
            if not hasattr(self, "_prev_vel_w"):
                self._prev_vel_w = self._asset.data.root_lin_vel_w.clone()

            vel_w = self._asset.data.root_lin_vel_w.clone()
            dv   = (vel_w - self._prev_vel_w)[0]  # first env
            self._prev_vel_w = vel_w
            current_mass = self._asset.root_physx_view.get_masses()[0].item()

            print(f"  Dynamic response: dv=({dv[0].item():.4f}, {dv[1].item():.4f}, {dv[2].item():.4f}) m/s")

        # ===== 5) APPLY aggregated wrench at CENTER OF MASS (3-DOF planar motion) =====
        rb_view = self._asset.root_physx_view
        
        # Apply the aggregated force and torque at the center of mass
        # This ensures no unwanted roll/pitch moments from lever arms
        rb_view.apply_forces_and_torques_at_position(
            force_data=total_forces_w,
            torque_data=total_torques_w,
            position_data=com_pos_w,
            indices=self._asset._ALL_INDICES,
            is_global=True,
        )



        # ===== 5) Planar velocity & attitude locks =====
        rb = self._asset.root_physx_view
        # NOTE: PhysX RigidBodyView does not expose setters for velocities in this build.
        # Skipping explicit velocity clamping to avoid attribute errors.
        # The planar behaviour is still enforced via zero Z force and torque locks.

        if self.cfg.level_body and hasattr(rb, "set_transforms"):
            # Re-level the body so orientation has yaw only (if API is available)
            if hasattr(rb, "set_transforms"):
                x_body = torch.tensor([1.0, 0.0, 0.0], device=self.device).expand(self.num_envs, -1)
                x_world = quat_apply(asset_quat_w, x_body)
                x_world[..., 2] = 0.0
                x_world = normalize(x_world)
                yaw = torch.atan2(x_world[..., 1], x_world[..., 0])
                euler = torch.zeros((self.num_envs, 3), device=self.device)
                euler[..., 2] = yaw
                R = matrix_from_euler(euler, convention="XYZ")
                q_yaw_only = quat_from_matrix(R)
                rb.set_transforms(positions=asset_pos_w, orientations=q_yaw_only, indices=self._asset._ALL_INDICES)

        ## Update fuel and mass
        self._remaining_fuel -= (
            self.cfg.fuel_consumption_rate
            * thrust_magnitudes.sum(dim=1)
            * self._env.cfg.agent_rate
        )
        self._remaining_fuel.clamp_(min=0.0)
        
        # --- COMMENTED OUT: Stop fighting mass override until we want fuel effects ---
        # masses = self._dry_masses + self._remaining_fuel.unsqueeze(-1)
        # mass_decrease_ratio = masses / self._asset.root_physx_view.get_masses()
        # self._asset.root_physx_view.set_masses(masses, indices=self._asset._ALL_INDICES)
        # self._asset.root_physx_view.set_inertias(
        #     mass_decrease_ratio * self._asset.root_physx_view.get_inertias(),
        #     indices=self._asset._ALL_INDICES,
        # )

        # ===== 7) (optional) update vis with WORLD directions already computed =====
        if self.cfg.debug_vis:
            self._update_visualization_markers(
                thruster_offsets=thruster_offsets_b,                     # still body offsets OK for marker
                thruster_directions=thruster_directions_b,               # marker function already transforms
                thrust_magnitudes=thrust_magnitudes,
            )

    def _setup_visualization_markers(self):
        # TODO[low]: Support custom visualization markers for thrusters
        self._thruster_markers = []
        for i in range(self._num_thrusters):
            cfg = ARROW_CFG.copy().replace(  # type: ignore
                prim_path=f"/Visuals/thrusters/thruster{i}"
            )
            cfg.markers["arrow"].tail_radius = 0.1
            cfg.markers["arrow"].tail_length = 1.0
            cfg.markers["arrow"].head_radius = 0.2
            cfg.markers["arrow"].head_length = 0.5

            # Use a different color for each thruster (gradient from red to blue)
            blue = i / max(self._num_thrusters - 1, 1)
            cfg.markers["arrow"].visual_material = PreviewSurfaceCfg(
                emissive_color=(1.0 - blue, 0.2, blue)
            )

            # Create the marker and add to list
            self._thruster_markers.append(VisualizationMarkers(cfg))

    def _update_visualization_markers(
        self,
        thruster_offsets: torch.Tensor,
        thruster_directions: torch.Tensor,
        thrust_magnitudes: torch.Tensor,
    ):
        asset_pos = self._asset.data.root_pos_w
        asset_quat = self._asset.data.root_quat_w

        for i in range(self._num_thrusters):
            # Transform thruster position to world frame
            thruster_pos_w, _ = combine_frame_transforms(
                t01=asset_pos,
                q01=asset_quat,
                t12=thruster_offsets[:, i, :],
            )

            # Orient the marker with the thrust vector
            thrust_dir_world = quat_apply(asset_quat, thruster_directions[:, i, :])
            thrust_dir_world = normalize(thrust_dir_world)

            # Create rotation matrix where x-axis is aligned with thrust direction
            x_axis = thrust_dir_world

            # Choose any perpendicular vector for y-axis
            y_axis = torch.zeros_like(x_axis)
            # Find index of smallest component in x_axis to create orthogonal vector
            min_idx = torch.argmin(torch.abs(x_axis), dim=1)
            for env_idx in range(self.num_envs):
                y_axis[env_idx, min_idx[env_idx]] = 1.0

            # Make y_axis perpendicular to x_axis
            y_axis = normalize(
                y_axis - x_axis * torch.sum(x_axis * y_axis, dim=1, keepdim=True)
            )

            # Get z_axis from cross product
            z_axis = normalize(torch.cross(x_axis, y_axis, dim=1))

            # Create rotation matrix
            rot_matrix = torch.zeros((self.num_envs, 3, 3), device=self.device)
            rot_matrix[:, :, 0] = x_axis
            rot_matrix[:, :, 1] = y_axis
            rot_matrix[:, :, 2] = z_axis

            # Convert to quaternion
            thrust_dir_quat_w = quat_from_matrix(rot_matrix)

            # Scale the marker based on the thrust magnitude
            marker_scale = torch.ones((self.num_envs, 3), device=self.device)
            marker_scale[:, :] = (
                thrust_magnitudes[:, i] * (1.0 / self._thruster_power.max().item())
            ).unsqueeze(1)

            # Visualize the marker
            self._thruster_markers[i].visualize(
                thruster_pos_w, thrust_dir_quat_w, marker_scale
            )

    def reset(self, env_ids: Sequence[int] | None = None):
        super().reset(env_ids)
        if env_ids is None:
            env_ids = torch.arange(self.num_envs, device=self.device)
        self._remaining_fuel[env_ids] = self.cfg.fuel_capacity
        if hasattr(self, "_prev_vel_w"):
            self._prev_vel_w[env_ids] = 0.0


class ThrusterCfg(BaseModel):
    offset: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    direction: Tuple[float, float, float] = (0.0, 0.0, -1.0)
    gimbal_limits: Tuple[float, float] | None = None
    power: float = 1.0


@configclass
class ThrustActionCfg(ActionTermCfg):
    class_type: Type = ThrustAction
    scale: float = 1.0

    thrusters: Sequence[ThrusterCfg] = (ThrusterCfg(),)
    fuel_capacity: float = 1.0
    fuel_consumption_rate: float = 0.1

    # --- Planar bring-up helpers ---
    planar_lock: bool = True        # Zero linear Z velocity each step
    lock_roll_pitch: bool = True    # Zero angular velocity in roll/pitch and cancel corresponding torques
    lock_yaw: bool = True           # Zero yaw angular velocity / torque for pure translation
    level_body: bool = False        # Force orientation to yaw-only each step (requires API availability)

    # --- XY force control allocator ---
    control_xy: bool = False        # If True, first 2 action dims are Fx,Fy commands in [-1,1]
    max_planar_force: float = 200.0 # Newtons for Fx,Fy normalisation
    allocator_lambda: float = 1e-3  # L2 regularisation
    allocator_steps: int = 20       # Gradient iterations
    allocator_lr: float = 0.6       # Gradient step size
