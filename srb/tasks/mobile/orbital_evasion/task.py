from dataclasses import MISSING
from typing import Sequence, Tuple

import torch
import numpy as np

from srb._typing.step_return import StepReturn
from srb.core.action import ThrustAction
from srb.core.asset import RigidObjectCollection, RigidObjectCollectionCfg
from srb.core.env.mobile import OrbitalEnv, OrbitalEnvCfg, OrbitalEventCfg, OrbitalSceneCfg
from srb.core.manager import EventTermCfg, SceneEntityCfg
from srb.core.marker import VisualizationMarkers, VisualizationMarkersCfg
from srb.core.mdp import reset_collection_root_state_uniform_poisson_disk_3d
from srb.core.sim import PreviewSurfaceCfg, SphereCfg
from srb.utils.cfg import configclass
from srb.utils.math import (
    deg_to_rad,
    matrix_from_quat,
    rotmat_to_rot6d,
    quat_from_two_vectors,
    quat_apply,              #  ← add this
)
from srb.core.env.mobile.orbital.visual_ext import (
    OrbitalEnvVisualExtCfg,
)

from .asset import select_obstacle
from srb.core.asset import RigidObjectCfg
from srb.core.sim import UsdFileCfg
from srb.core.sensor import Imu, ImuCfg


##############
### Config ###
##############

@configclass
class SceneCfg(OrbitalSceneCfg):
    env_spacing: float = 25.0
    num_envs: int = 1  # Single environment for maximum performance

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
    episode_length_s: float = 1e9  # Run for a very long time
    is_finite_horizon: bool = False
    # Ultra high-performance simulation rates
    env_rate: float = 1.0 / 500.0    # 500 Hz physics step - push the limits
    agent_rate: float = 1.0 / 500.0  # 500 Hz agent step (1:1 decimation)

    ## Performance optimizations for high-frequency simulation
    debug_vis: bool = False  # Disable debug visualization for performance
    
    ## Debug motion (meters & rad/step). 0 amp => fully static.
    debug_robot_orbit_amp_m: float = 0.0
    debug_robot_orbit_speed_rad: float = 0.0

    ## Target marker (disabled for maximum performance)
    tf_pos_target: Tuple[float, float, float] = (0.0, 0.0, -50.0)
    tf_quat_target: Tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)
    target_marker_cfg: VisualizationMarkersCfg | None = None  # Disabled for performance

    def __post_init__(self):
        # ULTRA aggressive PhysX optimizations for maximum performance
        if hasattr(self, 'sim') and hasattr(self.sim, 'physx'):
            # Minimal solver iterations
            self.sim.physx.min_position_iteration_count = 1
            self.sim.physx.min_velocity_iteration_count = 0  # Even more aggressive
            # Disable ALL expensive features
            self.sim.physx.enable_ccd = False
            self.sim.physx.enable_stabilization = False
            self.sim.physx.bounce_threshold_velocity = 0.0
            self.sim.physx.friction_offset_threshold = 0.0
            self.sim.physx.friction_correlation_distance = 0.0
        
        # Disable ALL rendering features
        if hasattr(self, 'sim') and hasattr(self.sim, 'render'):
            self.sim.render.enable_translucency = False
            self.sim.render.enable_reflections = False
        # 🛰️ Use standard robot prim path for proper IMU attachment
        # Custom prim paths can cause IMU sensor initialization failures
        # if hasattr(self, 'robot') and hasattr(self.robot, 'asset_cfg'):
        #     self.robot.asset_cfg.prim_path = "{ENV_REGEX_NS}/lab_sc_robot_unique_20250730"

        # 🛰️ IMU ENABLED - Keep IMU sensor active for real sensor data
        # if hasattr(self, 'scene') and hasattr(self.scene, 'imu_robot'):
        #     self.scene.imu_robot = None  # <- This line disabled IMU, now commented out
        
        # 🛰️ Ensure IMU sensor configured and pointing to robot root
        from srb.core.sensor import ImuCfg as _ImuCfg
        self.scene.imu_robot = _ImuCfg(prim_path="{ENV_REGEX_NS}/robot")
        # Target IMU rate - 200 Hz for high-performance system
        try:
            self.scene.imu_robot.update_period = 1.0 / 20.0
        except Exception:
            pass
        print("[🛰️ IMU FIX] IMU attached to /robot (with RigidBodyAPI) instead of /robot/base")


        # 📷 Force-enable Isaac Lab cameras via carb settings (CLI may not expose --enable_cameras)
        try:
            import carb  # type: ignore
            settings = carb.settings.get_settings()
            settings.set("/isaaclab/cameras_enabled", True)
            settings.set("/isaaclab/cameras_render", True)
            print("[📷 CAM] Forced cameras_enabled=True in carb settings")
        except Exception as e:
            print(f"[📷 CAM] Could not set camera settings via carb: {e}")

        # Completely disable robot randomization - we'll set initial state directly
        self.events.randomize_robot_state = None
        
        # Call the parent's __post_init__ which will set up the robot
        super().__post_init__()

        # 📷 Onboard camera — 30 FPS, RGB only, small resolution
        try:
            _visual_ext = OrbitalEnvVisualExtCfg()

            # rate
            if hasattr(_visual_ext, "camera_framerate"):
                _visual_ext.camera_framerate = 30.0
            if hasattr(_visual_ext, "camera_dt"):
                _visual_ext.camera_dt = 1.0 / 30.0

            # resolution (keep it small)
            if hasattr(_visual_ext, "camera_resolution"):
                _visual_ext.camera_resolution = (64, 64)
            if hasattr(_visual_ext, "camera_width"):
                _visual_ext.camera_width = 64
            if hasattr(_visual_ext, "camera_height"):
                _visual_ext.camera_height = 64

            # disable depth & pointcloud for now
            for attr in ("enable_depth", "publish_depth", "depth_enabled"):
                if hasattr(_visual_ext, attr):
                    setattr(_visual_ext, attr, False)
            for attr in ("enable_pointcloud", "publish_pointcloud", "pointcloud_enabled"):
                if hasattr(_visual_ext, attr):
                    setattr(_visual_ext, attr, False)

            _visual_ext.wrap(env_cfg=self)
            print("[📷 CAM] RGB 64x64 @30 FPS (depth/pointcloud OFF)")
        except Exception as e:
            print(f"[📷 CAM] Onboard camera configuration failed: {e}")

        # 🛰️ IMU — bind to robot root and run at 200 Hz
        try:
            imu_period = 1.0 / 200.0
            if hasattr(self.scene, 'imu_robot') and self.scene.imu_robot is not None:
                # self.scene.imu_robot is the *config*; adjust it so builder instantiates correctly
                self.scene.imu_robot.prim_path = self.scene.robot.prim_path
                if hasattr(self.scene.imu_robot, "update_period"):
                    self.scene.imu_robot.update_period = imu_period
                print("[🛰️ IMU] Config bound to robot root @200 Hz")
            else:
                # If no IMU cfg was present, create one now
                from srb.core.sensor import ImuCfg as _ImuCfg
                self.scene.imu_robot = _ImuCfg(prim_path=self.scene.robot.prim_path)
                if hasattr(self.scene.imu_robot, "update_period"):
                    self.scene.imu_robot.update_period = imu_period
                print("[🛰️ IMU] Created IMU config @200 Hz")
        except Exception as e:
            print(f"[🛰️ IMU] Failed to set 200 Hz IMU config: {e}")

        # (Optional while debugging control) disable auto camera motion baseline
        # if hasattr(self, 'enable_translational_motion'):
        #     self.enable_translational_motion = False
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
                    pos=(-100.0, 0.0, 0.0),
                    rot=(1.0, 0.0, 0.0, 0.0),
                ),
            )
        }

        # refresh any procedural assets (safe even with empty object list)
        self._update_procedural_assets()
        
        print("[🛰️ IMU ENABLED] Robot spawning completed with IMU sensor!")
        print(f"[🛰️ IMU ENABLED] Using standard robot prim path for IMU compatibility")
        print(f"[🛰️ IMU ENABLED] Real IMU data should be published on: /srb/env0/imu_robot")
        if hasattr(self.robot.asset_cfg, 'init_state') and self.robot.asset_cfg.init_state:
            print(f"[🛰️ IMU ENABLED] Configured position: {self.robot.asset_cfg.init_state.pos}")
            print(f"[🛰️ IMU ENABLED] Configured rotation: {self.robot.asset_cfg.init_state.rot}")


############
### Task ###
############

class Task(OrbitalEnv):
    cfg: TaskCfg

    def __init__(self, cfg: TaskCfg, **kwargs):
        import numpy as np
        print("[TASK] Initializing regular Task (not VisualTask)...")
        super().__init__(cfg, **kwargs)
        # Finalize IMU binding: force IMU to attach to the robot root and run at 200 Hz
        try:
            imu_target_period = 1.0 / 200.0

            robot_prim = "{ENV_REGEX_NS}/robot"
            if hasattr(self, "scene") and hasattr(self.scene, "robot") and hasattr(self.scene.robot, "prim_path"):
                robot_prim = self.scene.robot.prim_path

            if getattr(self, "_imu_robot", None) is not None:
                if hasattr(self._imu_robot, "cfg"):
                    self._imu_robot.cfg.prim_path = robot_prim
                    if hasattr(self._imu_robot.cfg, "update_period"):
                        self._imu_robot.cfg.update_period = imu_target_period
                if hasattr(self._imu_robot, "update_period"):
                    self._imu_robot.update_period = imu_target_period
                print(f"[🛰️ IMU FIX FINAL] Bound IMU to '{robot_prim}' @200 Hz")
            else:
                imu_cfg = ImuCfg(prim_path=robot_prim)
                if hasattr(imu_cfg, "update_period"):
                    imu_cfg.update_period = imu_target_period
                self.scene["imu_robot"] = Imu(imu_cfg)
                self._imu_robot = self.scene.get("imu_robot")
                print(f"[🛰️ IMU CREATE] Instantiated IMU at '{robot_prim}' @200 Hz")
        except Exception as e:
            print(f"[🛰️ IMU FIX FINAL] Could not rebind/create IMU: {e}")
        self._objs: RigidObjectCollection = self.scene["objs"]
        self._orbit_angle = 0.0
        self._orbit_radius = 150.0  # 75 meters away from Apophis
        self._orbit_center = torch.tensor([-100.0, 0.0, 0.0], device=self.device)  # Z fixed to match Apophis
        self._orbit_angular_speed = 0.0005  # radians per step, slower orbit for easier following
        self._debug_counter = 0  # Counter to control debug print frequency
        self._dbg_phase = 0.0  # Debug motion phase
        
        # Initialize PID controller state variables
        self._prev_pos_error = torch.zeros(3, device=self.device)
        self._integral_pos_error = torch.zeros(3, device=self.device)
        self._prev_yaw_error = 0.0
        self._integral_yaw_error = 0.0
        
        # Orbital motion parameters
        self._orbit_radius_pid = 50.0  # 50m orbit around Apophis for PID controller
        self._orbit_speed = 0.1   # rad/s orbital velocity
        self._start_time = None
        
        # Restore target marker and pose attributes (skip marker for performance)
        self._target_marker = None
        if self.cfg.target_marker_cfg is not None:
            self._target_marker = VisualizationMarkers(self.cfg.target_marker_cfg)
        self._tf_pos_target = (
            self.scene.env_origins
            + torch.tensor(self.cfg.tf_pos_target, device=self.device)
        ).repeat(self.num_envs, 1)
        self._tf_quat_target = torch.tensor(
            self.cfg.tf_quat_target, device=self.device
        ).repeat(self.num_envs, 1)
        
        print("[🛰️ IMU ENABLED] PID orbital controller initialized")
        if self._target_marker is not None:
            self._target_marker.visualize(self._tf_pos_target, self._tf_quat_target)
        
        # Check if IMU is actually available
        if hasattr(self, '_imu_robot') and self._imu_robot is not None:
            print("✅ [🛰️ IMU ENABLED] IMU sensor is available!")
            print("📡 [🛰️ IMU ENABLED] Real IMU data will be published on: /srb/env0/imu_robot")
        else:
            print("❌ [🛰️ IMU ENABLED] Warning: IMU sensor not found!")
        # Print thruster configuration at startup
        if hasattr(self, 'action_manager') and self.action_manager is not None:
            for term_name, term in self.action_manager._terms.items():
                if hasattr(term, '_thruster_direction'):
                    print("\n=== Thruster Directions ===\n", term._thruster_direction)
                    print("=== Thruster Powers ===\n", term._thruster_power)
                if hasattr(term, '_thruster_offset'):
                    print("=== Thruster Offsets ===\n", term._thruster_offset)
                if hasattr(term, '_thruster_power'):
                    print("=== Thruster Powers ===\n", term._thruster_power)
        # Print spacecraft mass
        if hasattr(self, '_robot') and hasattr(self._robot, 'mass'):
            print(f"[DEBUG] Spacecraft mass: {self._robot.mass}")
            
        # Debug robot position immediately after creation
        if hasattr(self, '_robot') and self._robot is not None:
            print("[DEBUG POST-CREATE] Robot created. Checking actual position...")
            try:
                pos = self._robot.data.root_pos_w[0].cpu().numpy()
                quat = self._robot.data.root_quat_w[0].cpu().numpy()
                print(f"[DEBUG POST-CREATE] Actual robot position: ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})")
                print(f"[DEBUG POST-CREATE] Actual robot quaternion: ({quat[0]:.3f}, {quat[1]:.3f}, {quat[2]:.3f}, {quat[3]:.3f}) [w,x,y,z]")
                
                # Convert to Euler for easier reading
                import numpy as np
                from scipy.spatial.transform import Rotation as R
                r = R.from_quat([quat[1], quat[2], quat[3], quat[0]])  # scipy expects x,y,z,w
                euler = r.as_euler('xyz', degrees=True)
                print(f"[DEBUG POST-CREATE] Actual robot rotation: ({euler[0]:.1f}°, {euler[1]:.1f}°, {euler[2]:.1f}°) [X,Y,Z]")
            except Exception as e:
                print(f"[DEBUG POST-CREATE] Could not read robot position: {e}")
        
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
        # [DEBUG] Applied camera jitter: 0.001 rad
        # if hasattr(self, '_debug_cam_jitter_rad') and self._debug_cam_jitter_rad > 0.0:
        #     # Apply camera jitter here if needed in the future
        #     pass
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

        # 🛰️ Get real IMU data from Isaac Sim sensor
        if self._imu_robot is not None:
            imu_lin_acc = self._imu_robot.data.lin_acc_b
            imu_ang_vel = self._imu_robot.data.ang_vel_b
            
            # Debug: Print IMU data every 1000 steps (minimal overhead)
            if hasattr(self, '_debug_counter') and self._debug_counter % 1000 == 0:
                print(f"[🛰️ IMU DATA] Step {self._debug_counter}: Accel={imu_lin_acc[0].cpu().numpy()}")
                print(f"[🛰️ IMU DATA] Step {self._debug_counter}: Angular={imu_ang_vel[0].cpu().numpy()}")
        else:
            # Fallback to zeros if IMU not available (silent)
            imu_lin_acc = torch.zeros(1, 3, device=self.device)
            imu_ang_vel = torch.zeros(1, 3, device=self.device)
        
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
            imu_lin_acc=imu_lin_acc,
            imu_ang_vel=imu_ang_vel,
            remaining_fuel=remaining_fuel,
        )




    # Removed _pre_physics_step override to allow ROS action processing
    # Actions are now handled by the parent DirectEnv._pre_physics_step method

    def _post_physics_step(self):
        # Debug logging to track robot position and orientation
        if not hasattr(self, '_debug_counter'):
            self._debug_counter = 0
            
        self._debug_counter += 1
        
        # Log position and orientation very rarely
        if (self._debug_counter % 2000) == 0:
            pos = self._robot.data.root_pos_w[0].cpu().numpy()
            quat = self._robot.data.root_quat_w[0].cpu().numpy()  # w, x, y, z
            
            # Convert quaternion to Euler angles for easier reading
            import numpy as np
            from scipy.spatial.transform import Rotation as R
            r = R.from_quat([quat[1], quat[2], quat[3], quat[0]])  # scipy expects x,y,z,w
            euler = r.as_euler('xyz', degrees=True)
            
            print(f"[DEBUG Step {self._debug_counter}] Robot pos: ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})")
            print(f"[DEBUG Step {self._debug_counter}] Robot rot: ({euler[0]:.1f}°, {euler[1]:.1f}°, {euler[2]:.1f}°) [X,Y,Z]")
            print(f"[DEBUG Step {self._debug_counter}] Robot quat: ({quat[0]:.3f}, {quat[1]:.3f}, {quat[2]:.3f}, {quat[3]:.3f}) [w,x,y,z]")



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
