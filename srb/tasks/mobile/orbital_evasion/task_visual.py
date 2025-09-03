# tasks/mobile/orbital_evasion/task_visual.py

from typing import Dict, Sequence, Optional, Tuple
import numpy as np
import cv2
import torch

from srb.core.env.mobile import OrbitalEnvVisualExtCfg
from srb.core.env.common.extension.visual.impl import VisualExt
from srb.utils.cfg import configclass
from srb._typing.step_return import StepReturn

from .task import Task, TaskCfg


@configclass
class VisualTaskCfg(OrbitalEnvVisualExtCfg, TaskCfg):
    # Visual extension overrides for performance
    camera_resolution: Tuple[int, int] = (64, 64)  # Small resolution
    camera_framerate: float = 1.0 / 30.0  # 30 FPS (0.0333 period) for high-performance hardware
    camera_data_types: Tuple[str, ...] = ("rgb",)  # RGB only, no depth
    
    # Additional performance optimizations
    debug_vis: bool = False  # Disable debug visualization
    rerender_on_reset: bool = False  # Don't re-render on reset
    skydome: bool = False  # Disable skydome for performance
    
    # Debug / tuning params (Hydra overrides OK)
    debug_cam_jitter_rad: float = 0.0    # disable automatic jitter since user is moving manually
    min_inlier_count:     int   = 5      # min RANSAC inliers to accept pose
    ransac_thresh_px:     float = 1.5    # RANSAC pixel thresh
    enable_pose_estimation: bool = False   # <-- add this; off for now
    
    # Pose estimation parameters
    min_keyframe_gap:     int   = 3      # minimum frames between keyframes
    min_baseline_m:       float = 0.1    # minimum translation baseline (m)
    assumed_step_baseline_m: float = 0.2 # heuristic scale for translation
    max_rotation_deg:     float = 45.0   # maximum rotation between frames (relaxed)
    min_cosine_similarity: float = 0.3   # minimum similarity for pose acceptance (relaxed)
    
    # Static detection and scale handling
    static_motion_threshold_m: float = 0.01  # GT motion below this = static
    static_rotation_threshold_deg: float = 5.0  # max rotation in static mode (relaxed)
    min_parallax_px: float = 0.01  # minimum parallax for translation (normalized coords)
    max_epipolar_error_px: float = 0.1  # maximum epipolar error (normalized coords)
    min_svd_ratio: float = 0.05  # minimum SVD ratio for E matrix conditioning (relaxed)
    
    # Ground truth comparison
    enable_gt_comparison: bool = True    # compare with simulator ground truth
    
    # Motion control for baseline
    enable_translational_motion: bool = True  # add real translation for baseline
    motion_amplitude_m: float = 0.25      # increased amplitude for better baseline
    motion_frequency: int = 5            # more frequent motion

    def __post_init__(self):
        TaskCfg.__post_init__(self)
        OrbitalEnvVisualExtCfg.wrap(self, env_cfg=self)


class VisualTask(VisualExt, Task):
    cfg: VisualTaskCfg

    def __init__(self, cfg: VisualTaskCfg, **kwargs):
        Task.__init__(self, cfg, **kwargs)
        VisualExt.__init__(self, cfg, **kwargs)

        # ORB & matcher (optimized for small images)
        self._orb = cv2.ORB_create(
            nfeatures=1000,     # Increased for better feature detection
            scaleFactor=1.2,    # Slightly larger scale factor
            nlevels=10,         # More levels for better detection
            edgeThreshold=20,   # Larger edge threshold
            firstLevel=0,       # Start from original scale
            WTA_K=2,            # 2-point random sampling
            patchSize=31,       # Smaller patch size
            fastThreshold=5     # Lower threshold for more features
        )
        self._bf = cv2.BFMatcher(cv2.NORM_HAMMING)
        
        # Upscaled detection for better sub-pixel motion
        self._detection_scale = 2.0  # 2x upscaling for detection

        # Camera intrinsics - will be initialized with actual image size
        self._K = None
        self._focal = None
        self._pp = None
        self._intrinsics_initialized = False

        # Frame tracking
        self._frame_idx = 0
        self._prev_keyframe_idx = -1
        self._prev_gray: Optional[np.ndarray] = None
        self._prev_frame: Optional[np.ndarray] = None
        self._keyframe_attempts = 0  # Track attempts to find good keyframe
        
        # Pose tracking
        self._T_wc = np.eye(4, dtype=np.float64)  # World to camera transform
        self._last_good_pose = None  # (R, t, inliers)
        self._last_good_T_wc = np.eye(4, dtype=np.float64)
        self._initialized_from_gt = False  # Track if we've initialized from GT
        
        # Ground truth tracking
        self._gt_poses = []  # List of ground truth poses for comparison
        self._last_gt_pose = None  # Last ground truth pose for static detection
        self._is_static_mode = False  # Track if we're in static mode
        
        # Quality metrics
        self._pose_quality_history = []
        
        # Motion control
        self._motion_step_counter = 0
        
        # Cumulative rotation tracking for keyframe refresh
        self._cumulative_rot_deg_since_keyframe = 0.0
        
        # Rotation-only rate limiting
        self._last_rotation_only_frame = -999
        
        # Feature count tracking for diagnostics
        self._feat_prev = 0
        self._feat_curr = 0
        
        # GT window for better static detection
        from collections import deque
        self._gt_window = deque(maxlen=6)
        
        # Debug counter for rate limiting prints
        self._dbg_every = 200  # only print once every 200 steps

    def _init_camera_intrinsics(self, width: int, height: int):
        """Initialize camera intrinsics based on actual image dimensions."""
        if self._intrinsics_initialized and self._pp == (width/2, height/2):
            return
            
        print(f"[VISUAL] Initializing intrinsics for {width}x{height}")
        
        # Camera intrinsics for actual image size (64x64, not 640x480)
        w, h = width, height
        HFOV_deg = 60.0  # Horizontal field of view
        
        self._pp = (w / 2.0, h / 2.0)
        self._focal = (w / 2.0) / np.tan(np.deg2rad(HFOV_deg / 2.0))
        self._K = np.array([[self._focal, 0.0, self._pp[0]],
                            [0.0, self._focal, self._pp[1]],
                            [0.0,        0.0,          1.0]], dtype=np.float64)
        
        self._intrinsics_initialized = True

    def _make_SE3(self, R: np.ndarray, t: np.ndarray) -> np.ndarray:
        """Create SE(3) transformation matrix from rotation and translation."""
        T = np.eye(4, dtype=np.float64)
        T[:3, :3] = R
        T[:3, 3] = t.ravel()
        return T

    def _extract_SE3_components(self, T: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Extract rotation and translation from SE(3) matrix."""
        R = T[:3, :3].copy()
        t = T[:3, 3].copy()
        return R, t

    def _check_pose_quality(self, R: np.ndarray, t: np.ndarray, inliers: int, 
                           pts1: np.ndarray, pts2: np.ndarray, E: np.ndarray) -> Tuple[bool, dict]:
        """Check if pose estimate is reasonable with enhanced quality checks."""
        reasons = {}
        ok = True

        # Check rotation matrix properties
        det_R = np.linalg.det(R)
        if abs(det_R - 1.0) > 0.05:
            reasons['det'] = det_R
            ok = False
            
        # Check rotation magnitude (stricter in static mode)
        euler = self._rotation_matrix_to_euler_degrees(R)
        max_rot = np.max(np.abs(euler))
        max_rot_threshold = self.cfg.static_rotation_threshold_deg if self._is_static_mode else self.cfg.max_rotation_deg
        if max_rot > max_rot_threshold:
            reasons['rot_mag'] = max_rot
            ok = False
            
        # Check inlier count
        if inliers < self.cfg.min_inlier_count:
            reasons['inliers'] = inliers
            ok = False
            
        # Check parallax using P95 raw disparity instead of median normalized
        raw_disp_stats = self._compute_raw_disparity_stats(pts1, pts2)
        p95_disp = raw_disp_stats['p95']
        
        # Don't reject on parallax - just annotate for scale decision
        if p95_disp < 0.05:  # Very low disparity
            reasons['low_p95_disp'] = p95_disp
            # Don't set ok = False - let it pass but mark for scale attenuation
        
        # Check essential matrix conditioning
        if not self._check_essential_matrix_conditioning(E):
            reasons['E_cond'] = True
            ok = False
            
        # Check epipolar error
        epipolar_error = self._compute_epipolar_error(pts1, pts2, E)
        if epipolar_error > self.cfg.max_epipolar_error_px:
            reasons['epipolar'] = epipolar_error
            ok = False
            
        # Check for degenerate case: low disparity with large rotation
        if p95_disp < 0.05 and max_rot > 12.0:
            reasons['rot_disparity_degenerate'] = (p95_disp, max_rot)
            ok = False
            
        # Check consistency with last good pose (only after we have a few poses)
        if self._last_good_pose is not None and self._frame_idx > 10:
            R_last, t_last, _ = self._last_good_pose
            
            # Check rotation consistency
            R_diff = R @ R_last.T
            euler_diff = self._rotation_matrix_to_euler_degrees(R_diff)
            max_rot_diff = np.max(np.abs(euler_diff))
            if max_rot_diff > self.cfg.max_rotation_deg * 2:
                reasons['rot_consistency'] = max_rot_diff
                ok = False
                
            # Check translation direction consistency (only after we have stable poses)
            if self._frame_idx > 20:
                t_norm = t / (np.linalg.norm(t) + 1e-8)
                t_last_norm = t_last / (np.linalg.norm(t_last) + 1e-8)
                cosine_sim = np.dot(t_norm.ravel(), t_last_norm.ravel())
                if abs(cosine_sim) < self.cfg.min_cosine_similarity:
                    reasons['trans_consistency'] = cosine_sim
                    ok = False
                
        # Return quality metrics for logging
        metrics = {
            'p95_raw_disp': p95_disp, 
            'epipolar': epipolar_error, 
            'inliers': inliers,
            'rot_deg': euler,
            **reasons
        }
        
        return ok, metrics

    def _normalize_points(self, pts: np.ndarray) -> np.ndarray:
        """Normalize pixel coordinates using camera intrinsics."""
        # pts: Nx2 pixel coordinates
        if self._K is None:
            raise ValueError("Camera intrinsics not initialized")
        Kinv = np.linalg.inv(self._K)
        pts_h = np.hstack([pts, np.ones((pts.shape[0], 1))])
        pts_norm = (Kinv @ pts_h.T).T
        return pts_norm[:, :2]

    def _compute_raw_disparity_stats(self, pts1: np.ndarray, pts2: np.ndarray) -> dict:
        """Compute raw pixel disparity statistics before normalization."""
        if pts1 is None or pts2 is None or len(pts1) == 0:
            return {'median': 0.0, 'p95': 0.0, 'mean': 0.0, 'std': 0.0}
            
        # Compute raw pixel displacements
        disp_values = np.linalg.norm(pts1 - pts2, axis=1)
        
        return {
            'median': float(np.median(disp_values)),
            'p95': float(np.percentile(disp_values, 95)),
            'mean': float(np.mean(disp_values)),
            'std': float(np.std(disp_values))
        }

    def _compute_parallax(self, pts1: np.ndarray, pts2: np.ndarray, normalized: bool = True) -> float:
        """Compute parallax between matched points."""
        if normalized:
            pts1n = self._normalize_points(pts1)
            pts2n = self._normalize_points(pts2)
            distances = np.linalg.norm(pts1n - pts2n, axis=1)
        else:
            distances = np.linalg.norm(pts1 - pts2, axis=1)
        parallax = float(np.median(distances))
        return parallax

    def _compute_epipolar_error(self, pts1: np.ndarray, pts2: np.ndarray, E: np.ndarray) -> float:
        """Compute symmetric epipolar error using normalized coordinates."""
        # Use normalized points for E-based calculations
        pts1n = self._normalize_points(pts1)
        pts2n = self._normalize_points(pts2)
        
        # Convert to homogeneous coordinates
        pts1_h = np.hstack([pts1n, np.ones((pts1n.shape[0], 1))])
        pts2_h = np.hstack([pts2n, np.ones((pts2n.shape[0], 1))])
        
        # Compute epipolar lines
        Ex1 = E @ pts1_h.T  # 3xN
        Etx2 = E.T @ pts2_h.T  # 3xN
        
        # Compute symmetric epipolar error
        x2tEx1 = np.sum(pts2_h * (E @ pts1_h.T).T, axis=1)
        d = x2tEx1**2 * (
            1.0 / (Ex1[0]**2 + Ex1[1]**2 + 1e-12) +
            1.0 / (Etx2[0]**2 + Etx2[1]**2 + 1e-12)
        )
        
        return float(np.median(d)**0.5)

    def _check_essential_matrix_conditioning(self, E: np.ndarray) -> bool:
        """Check essential matrix conditioning via SVD."""
        U, S, Vt = np.linalg.svd(E)
        
        # Essential matrix should have two similar non-zero singular values and one near zero
        if S[2] > 1e-4 * S[0]:  # Third singular value should be very small
            return False
            
        if abs(S[0] - S[1]) / S[0] > 0.15:  # First two should be similar
            return False
            
        return True

    def _evaluate_static(self, current_gt: np.ndarray) -> bool:
        """Evaluate if camera is in static mode without updating last GT pose."""
        if self._last_gt_pose is None:
            return False
            
        # Compute motion between current and last GT pose
        R_gt, t_gt = self._extract_SE3_components(current_gt)
        R_last, t_last = self._extract_SE3_components(self._last_gt_pose)
        
        # Check translation
        translation_magnitude = np.linalg.norm(t_gt - t_last)
        
        # Check rotation
        R_diff = R_gt @ R_last.T
        euler_diff = self._rotation_matrix_to_euler_degrees(R_diff)
        rotation_magnitude = np.max(np.abs(euler_diff))
        
        # Static if both translation and rotation are small
        return (translation_magnitude < self.cfg.static_motion_threshold_m and 
                rotation_magnitude < self.cfg.static_rotation_threshold_deg)

    def _detect_static_motion(self) -> bool:
        """Detect if camera is in static mode based on GT motion."""
        if not self.cfg.enable_gt_comparison:
            return False
            
        T_gt = self._get_ground_truth_pose()
        if T_gt is None or self._last_gt_pose is None:
            return False
            
        # Compute motion between current and last GT pose
        R_gt, t_gt = self._extract_SE3_components(T_gt)
        R_last, t_last = self._extract_SE3_components(self._last_gt_pose)
        
        # Check translation
        translation_magnitude = np.linalg.norm(t_gt - t_last)
        
        # Check rotation
        R_diff = R_gt @ R_last.T
        euler_diff = self._rotation_matrix_to_euler_degrees(R_diff)
        rotation_magnitude = np.max(np.abs(euler_diff))
        
        # Update last GT pose
        self._last_gt_pose = T_gt.copy()
        
        # Static if both translation and rotation are small
        return (translation_magnitude < self.cfg.static_motion_threshold_m and 
                rotation_magnitude < self.cfg.static_rotation_threshold_deg)

    def _get_scale_from_gt(self, force_allow: bool = False) -> float:
        """Get scale from ground truth motion if available."""
        if not self.cfg.enable_gt_comparison or self._last_gt_pose is None:
            return self.cfg.assumed_step_baseline_m
            
        T_gt = self._get_ground_truth_pose()
        if T_gt is None:
            return self.cfg.assumed_step_baseline_m
            
        # Compute GT motion
        R_gt, t_gt = self._extract_SE3_components(T_gt)
        R_last, t_last = self._extract_SE3_components(self._last_gt_pose)
        
        gt_translation = np.linalg.norm(t_gt - t_last)
        
        # Always return heuristic scale for micro motion, not 0
        if gt_translation > self.cfg.static_motion_threshold_m or force_allow:
            return float(gt_translation)
        else:
            # Micro motion – return heuristic (do NOT return 0 unless rotation-only)
            return self.cfg.assumed_step_baseline_m

    def _get_ground_truth_pose(self) -> Optional[np.ndarray]:
        """Get ground truth camera pose from simulator."""
        try:
            # Try different possible camera attribute names
            camera_attr = None
            for attr_name in ['camera_onboard', 'camera', 'cam', 'onboard_camera']:
                if hasattr(self, attr_name):
                    camera_attr = getattr(self, attr_name)
                    break
            
            if camera_attr is None:
                # Try to get from robot data
                if hasattr(self, '_robot') and hasattr(self._robot, 'data'):
                    # Use robot pose as camera pose (approximation)
                    pos_w = self._robot.data.root_pos_w[0].cpu().numpy()
                    quat_w = self._robot.data.root_quat_w[0].cpu().numpy()
                else:
                    return None
            else:
                pos_w = camera_attr.pos_w[0].cpu().numpy()
                quat_w = camera_attr.quat_w[0].cpu().numpy()
            
            # Convert quaternion to rotation matrix
            qw, qx, qy, qz = quat_w
            R = np.array([
                [1-2*qy*qy-2*qz*qz, 2*(qx*qy-qw*qz), 2*(qx*qz+qw*qy)],
                [2*(qx*qy+qw*qz), 1-2*qx*qx-2*qz*qz, 2*(qy*qz-qw*qx)],
                [2*(qx*qz-qw*qy), 2*(qy*qz+qw*qx), 1-2*qx*qx-2*qy*qy]
            ], dtype=np.float64)
            
            T_wc = self._make_SE3(R, pos_w)
            return T_wc
            
        except Exception as e:
            print(f"[GT] Error getting ground truth pose: {e}")
            return None

    def _get_asteroid_relative_pose(self, T_wc: np.ndarray) -> Optional[Tuple[np.ndarray, float]]:
        """Get asteroid pose relative to camera."""
        try:
            # Get asteroid world position
            ast_pos_w = self._objs.data.object_com_pos_w[0, 0].cpu().numpy()
            
            # Transform to camera frame
            R_wc, t_wc = self._extract_SE3_components(T_wc)
            ast_pos_c = R_wc.T @ (ast_pos_w - t_wc)
            
            # Distance to asteroid
            distance = float(np.linalg.norm(ast_pos_c))
            
            return ast_pos_c, distance
            
        except Exception as e:
            print(f"[GT] Error getting asteroid relative pose: {e}")
            return None

    # ------------------------------------------------------------------ #
    # Merge Task + VisualExt observations                                #
    # ------------------------------------------------------------------ #
    def _get_observations(self) -> Dict[str, torch.Tensor]:
        return {**Task._get_observations(self), **VisualExt._get_observations(self)}

    # ------------------------------------------------------------------ #
    # Extract step return - called every step by SRB framework           #
    # ------------------------------------------------------------------ #
    def extract_step_return(self) -> StepReturn:
        # PERFORMANCE: Skip ALL visual processing for maximum speed
        # Just get visual observations without processing them
        try:
            # Get visual observations but don't process them (for ROS publishing only)
            visual_obs = VisualExt._get_observations(self)
            # Skip all image processing, pose estimation, etc.
        except Exception as e:
            print(f"[VISUAL] Error in extract_step_return: {e}")
        
        # Always return the Task step (publishes IMU, etc.)
        return Task.extract_step_return(self)

    def _process_frame_for_pose(self, gray: np.ndarray, img: np.ndarray):
        """Process frame for pose estimation with keyframe selection."""
        # Check if we should use this as a keyframe
        frame_gap = self._frame_idx - self._prev_keyframe_idx
        
        # Handle case where we have a previous keyframe and sufficient gap
        if self._prev_gray is not None and frame_gap >= self.cfg.min_keyframe_gap:
            # Check baseline between frames using keypoint displacement
            baseline_ok = self._has_sufficient_baseline(self._prev_gray, gray)
            
            if baseline_ok:
                # Debug: Print pose estimation attempt
                if (self._frame_idx % self._dbg_every) == 0:
                    print(f"[DEBUG] Frame {self._frame_idx}: Attempting pose estimation")
                
                # Check feature count before attempting pose estimation
                kp_curr, des_curr = self._orb.detectAndCompute(gray, None)
                if kp_curr is None or len(kp_curr) < 20:
                    if (self._frame_idx % self._dbg_every) == 0:
                        print(f"[DEBUG] Frame {self._frame_idx}: Skipping pose (features={0 if kp_curr is None else len(kp_curr)})")
                    return
                
                # Estimate relative pose
                R_rel, t_dir, inliers, pts1, pts2, E = self._estimate_relative_pose(self._prev_gray, gray)
                
                # Debug: Print pose estimation results
                if (self._frame_idx % self._dbg_every) == 0:
                    print(f"[DEBUG] Frame {self._frame_idx}: Pose estimation result - R_rel={R_rel is not None}, t_dir={t_dir is not None}, inliers={inliers}, pts1={pts1 is not None}, pts2={pts2 is not None}, E={E is not None}")
                
                quality_check = False
                rotation_only = False
                quality_metrics = {}
                
                if R_rel is not None and t_dir is not None and pts1 is not None and pts2 is not None and E is not None:
                    # Update cumulative rotation tracking with proper magnitude
                    euler_rel = self._rotation_matrix_to_euler_degrees(R_rel)
                    rot_inc = np.linalg.norm(euler_rel)  # Use magnitude instead of max absolute
                    
                    # Disable rotation accumulation in low disparity scenarios
                    raw_disp_stats = self._compute_raw_disparity_stats(pts1, pts2)
                    if raw_disp_stats['p95'] < 0.3:  # Low disparity: ignore rotation accumulation
                        rot_inc = 0.0
                    
                    # Clamp rotation increment to avoid noise accumulation
                    if rot_inc < 0.01:
                        rot_inc = 0.0
                        
                    self._cumulative_rot_deg_since_keyframe += rot_inc
                    self._cumulative_rot_deg_since_keyframe = float(min(self._cumulative_rot_deg_since_keyframe, 90.0))
                    
                    # Check for keyframe refresh due to rotation accumulation with adaptive threshold
                    rot_refresh_threshold = 60.0 if raw_disp_stats['p95'] < 0.3 else (25.0 if not self._is_static_mode else 35.0)
                    if self._cumulative_rot_deg_since_keyframe > rot_refresh_threshold:
                        # Check if current frame has enough features for new keyframe
                        kp_curr, _ = self._orb.detectAndCompute(gray, None)
                        feature_count = len(kp_curr) if kp_curr is not None else 0
                        if feature_count >= 20:
                            print(f"[RECOVERY] Frame {self._frame_idx}: Keyframe refreshed due to rotation accumulation ({self._cumulative_rot_deg_since_keyframe:.1f}°)")
                            self._prev_keyframe_idx = self._frame_idx
                            self._prev_gray = gray.copy()
                            self._prev_frame = img.copy()
                            self._cumulative_rot_deg_since_keyframe = 0.0
                            return
                    
                    # Evaluate static mode before quality check
                    if self.cfg.enable_gt_comparison and self._last_gt_pose is not None:
                        current_gt = self._get_ground_truth_pose()
                        if current_gt is not None:
                            self._is_static_mode = self._evaluate_static(current_gt)
                    
                    # Check pose quality
                    quality_check, quality_metrics = self._check_pose_quality(R_rel, t_dir, inliers, pts1, pts2, E)
                    
                    # Enhanced rotation-only acceptance with proper guards
                    if (not quality_check 
                        and 'p95_raw_disp' in quality_metrics 
                        and quality_metrics['p95_raw_disp'] < 0.05):
                        
                        # Rotation-only constraints
                        euler = quality_metrics['rot_deg']
                        max_abs_rot = np.max(np.abs(euler))
                        rot_jump_ok = True
                        
                        if self._last_good_pose is not None:
                            R_last, _, _ = self._last_good_pose
                            R_diff = R_rel @ R_last.T
                            euler_diff = self._rotation_matrix_to_euler_degrees(R_diff)
                            rot_jump = np.max(np.abs(euler_diff))
                            rot_jump_ok = rot_jump < (15.0 if not self._is_static_mode else 8.0)
                        
                        rot_only_limit = 10.0 if not self._is_static_mode else 5.0
                        
                        if max_abs_rot < rot_only_limit and rot_jump_ok:
                            t_dir = np.zeros_like(t_dir)
                            quality_check = True
                            rotation_only = True
                        else:
                            quality_metrics['rot_only_reject'] = {
                                'max_abs_rot': float(max_abs_rot),
                                'rot_jump_ok': rot_jump_ok,
                                'rot_jump_limit': 15.0 if not self._is_static_mode else 8.0
                            }
                    
                    # Debug: Print quality check result
                    if (self._frame_idx % self._dbg_every) == 0:
                        print(f"[DEBUG] Frame {self._frame_idx}: Quality check result = {quality_check}, rotation_only = {rotation_only}, static_mode = {self._is_static_mode}")
                        if not quality_check:
                            print(f"[REJECT] Frame {self._frame_idx} reasons: {quality_metrics}")
                
                if R_rel is not None and t_dir is not None and pts1 is not None and pts2 is not None and E is not None and quality_check:
                    # Apply rotation-only rate limiting
                    if rotation_only:
                        if self._last_rotation_only_frame >= 0 and (self._frame_idx - self._last_rotation_only_frame) < 15:
                            # Skip integrating repeated rotation-only updates
                            if (self._frame_idx % self._dbg_every) == 0:
                                print(f"[DEBUG] Frame {self._frame_idx}: Skipping rotation-only (rate limited)")
                            return
                        self._last_rotation_only_frame = self._frame_idx
                    
                    # Determine scale based on P95 disparity buckets
                    if rotation_only:
                        scale = 0.0  # No translation for rotation-only poses
                    else:
                        # Get raw disparity stats for scale decision
                        raw_disp_stats = self._compute_raw_disparity_stats(pts1, pts2)
                        p95_disp = raw_disp_stats['p95']
                        
                        if p95_disp < 0.05:  # Stricter tiny threshold
                            # Too little disparity - rotation only
                            scale = 0.0
                        elif p95_disp < 0.15:
                            # Micro-baseline: attenuated scale
                            scale = 0.25 * self._get_scale_from_gt(force_allow=True)
                        elif p95_disp < 0.40:
                            # Medium-baseline: half scale
                            scale = 0.5 * self._get_scale_from_gt(force_allow=True)
                        else:
                            # Normal baseline: full scale
                            scale = self._get_scale_from_gt(force_allow=True)
                    
                    # Debug scale decision
                    if (self._frame_idx % self._dbg_every) == 0:
                        print(f"[SCALE_DBG] Frame {self._frame_idx}: p95={p95_disp:.3f} gt_scale={self._get_scale_from_gt(force_allow=True):.4f} is_static={self._is_static_mode} final_scale={scale:.4f}")
                    
                    # Apply scale
                    t_scaled = scale * t_dir
                    
                    # Create relative transformation
                    T_rel = self._make_SE3(R_rel, t_scaled)
                    
                    # Initialize from ground truth on first successful pose
                    if not self._initialized_from_gt and self.cfg.enable_gt_comparison:
                        T_gt = self._get_ground_truth_pose()
                        if T_gt is not None:
                            self._T_wc = T_gt.copy()
                            self._initialized_from_gt = True
                            self._last_gt_pose = T_gt.copy()
                            scale = 0.0  # override bogus first scale
                            print(f"[POSE] Initialized pose from ground truth at frame {self._frame_idx}")
                        else:
                            # Accumulate global pose normally
                            self._T_wc = self._T_wc @ T_rel
                    else:
                        # Accumulate global pose
                        self._T_wc = self._T_wc @ T_rel
                    
                    # Store as last good pose
                    self._last_good_pose = (R_rel.copy(), t_dir.copy(), inliers)
                    self._last_good_T_wc = self._T_wc.copy()
                    
                    # Update GT pose after acceptance
                    if self.cfg.enable_gt_comparison:
                        current_gt = self._get_ground_truth_pose()
                        if current_gt is not None:
                            self._last_gt_pose = current_gt.copy()
                    
                    # Reset cumulative rotation tracking
                    self._cumulative_rot_deg_since_keyframe = 0.0
                    
                    # Log pose information with rotation-only indicator
                    self._log_pose_estimation(inliers, frame_gap, pts1, pts2, E, scale, rotation_only)
                    
                    # Update keyframe
                    self._prev_keyframe_idx = self._frame_idx
                    self._prev_gray = gray.copy()
                    self._prev_frame = img.copy()
                else:
                    # Debug: Print rejection reasons
                    if (self._frame_idx % self._dbg_every) == 0:
                        print(f"[DEBUG] Frame {self._frame_idx}: Pose rejected - R_rel={R_rel is not None}, t_dir={t_dir is not None}, pts1={pts1 is not None}, pts2={pts2 is not None}, E={E is not None}")
            else:
                # Debug: Print baseline rejection
                if (self._frame_idx % self._dbg_every) == 0:
                    print(f"[DEBUG] Frame {self._frame_idx}: Insufficient baseline")
            return
        
        else:
            # Check if we need to reset keyframe due to feature loss
            if self._prev_gray is not None:
                # Check current frame features
                kp, des = self._orb.detectAndCompute(gray, None)
                feature_count = len(kp) if kp is not None else 0
                
                # If we have features but keyframe is too old, reset
                if feature_count >= 20 and (self._frame_idx - self._prev_keyframe_idx) > 50:
                    print(f"[RECOVERY] Frame {self._frame_idx}: Resetting keyframe (old: {self._prev_keyframe_idx}, features: {feature_count})")
                    self._prev_keyframe_idx = self._frame_idx
                    self._prev_gray = gray.copy()
                    self._prev_frame = img.copy()
                    self._keyframe_attempts = 0
                    return
                
                # Handle large motion as keyframe opportunity
                if frame_gap >= self.cfg.min_keyframe_gap:
                    # Check if this is a large motion frame that could be a good keyframe
                    diff = np.mean(np.abs(gray.astype(np.float32) - self._prev_gray.astype(np.float32)))
                    if diff > 50.0 and feature_count >= 40:  # Large motion with good features
                        print(f"[RECOVERY] Frame {self._frame_idx}: High-motion keyframe reset (diff={diff:.1f}, features={feature_count})")
                        self._prev_keyframe_idx = self._frame_idx
                        self._prev_gray = gray.copy()
                        self._prev_frame = img.copy()
                        self._cumulative_rot_deg_since_keyframe = 0.0
                        self._keyframe_attempts = 0
                        return
        
        # Initialize keyframe - only if current frame has sufficient features
        if self._prev_gray is None:
            # Check if current frame has enough features before using as keyframe
            kp, des = self._orb.detectAndCompute(gray, None)
            feature_count = len(kp) if kp is not None else 0
            
            if feature_count >= 15:  # Lower threshold for manual motion
                self._prev_keyframe_idx = self._frame_idx
                self._prev_gray = gray.copy()
                self._prev_frame = img.copy()
                self._keyframe_attempts = 0  # Reset attempts
                print(f"[RECOVERY] Frame {self._frame_idx}: Recovered with {feature_count} features")
            else:
                self._keyframe_attempts += 1
                
                # If we've tried too many times, use the best frame we've seen
                if self._keyframe_attempts >= 5:  # Faster recovery for manual motion
                    print(f"[WARNING] Frame {self._frame_idx}: Using frame with only {feature_count} features as keyframe")
                    self._prev_keyframe_idx = self._frame_idx
                    self._prev_gray = gray.copy()
                    self._prev_frame = img.copy()
                    self._keyframe_attempts = 0

    def _has_sufficient_baseline(self, prev_gray: np.ndarray, curr_gray: np.ndarray) -> bool:
        """Check if there's sufficient baseline using keypoint displacement with upscaled detection."""
        # Upscale for better feature detection
        det_prev_gray = cv2.resize(prev_gray, None, fx=self._detection_scale, fy=self._detection_scale, interpolation=cv2.INTER_CUBIC)
        det_curr_gray = cv2.resize(curr_gray, None, fx=self._detection_scale, fy=self._detection_scale, interpolation=cv2.INTER_CUBIC)
        
        kp1, des1 = self._orb.detectAndCompute(det_prev_gray, None)
        kp2, des2 = self._orb.detectAndCompute(det_curr_gray, None)
        
        if (des1 is None or des2 is None or 
            len(kp1) < 25 or len(kp2) < 25):
            return False
            
        # Use BF matcher for displacement calculation
        matches = self._bf.match(des1, des2)
        if len(matches) < 20:
            return False
            
        # Calculate median displacement (scale back to original resolution)
        displacements = []
        for m in matches:
            pt1 = np.array(kp1[m.queryIdx].pt) / self._detection_scale
            pt2 = np.array(kp2[m.trainIdx].pt) / self._detection_scale
            disp = np.linalg.norm(pt1 - pt2)
            displacements.append(disp)
            
        median_disp = np.median(displacements)
        
        # Allow smaller displacements for micro-baseline regime
        min_disp = 0.15  # pixels
        max_disp = 30.0  # pixels (allow larger motion)
        
        return bool(min_disp <= median_disp <= max_disp)

    def _check_baseline_sufficient(self, gray1: np.ndarray, gray2: np.ndarray) -> bool:
        """Check if there's sufficient baseline between frames for reliable pose estimation."""
        # Simple check: compute mean absolute difference
        diff = np.mean(np.abs(gray1.astype(np.float32) - gray2.astype(np.float32)))
        
        # Adaptive threshold for manual motion
        min_threshold = 0.5   # Minimum difference needed
        max_threshold = 50.0  # Maximum difference (avoid motion blur)
        
        if diff < min_threshold:
            return False
        elif diff > max_threshold:
            # Too much motion - might cause motion blur
            if self._frame_idx % 20 == 0:  # Log occasionally
                print(f"[WARNING] Frame {self._frame_idx}: Motion too large (diff={diff:.1f}), might cause tracking issues")
            return False
            
        return True

    def _estimate_relative_pose(self, gray1: np.ndarray, gray2: np.ndarray) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], int, Optional[np.ndarray], Optional[np.ndarray], Optional[np.ndarray]]:
        """Estimate relative pose between two frames using 5-point algorithm with upscaled detection."""
        # Upscale for better feature detection
        det_gray1 = cv2.resize(gray1, None, fx=self._detection_scale, fy=self._detection_scale, interpolation=cv2.INTER_CUBIC)
        det_gray2 = cv2.resize(gray2, None, fx=self._detection_scale, fy=self._detection_scale, interpolation=cv2.INTER_CUBIC)
        
        # Detect ORB features on upscaled images
        kp1, des1 = self._orb.detectAndCompute(det_gray1, None)
        kp2, des2 = self._orb.detectAndCompute(det_gray2, None)

        if des1 is None or des2 is None or len(kp1) < 5 or len(kp2) < 5:
            return None, None, 0, None, None, None

        # Track feature counts for diagnostics
        self._feat_prev = len(kp1)
        self._feat_curr = len(kp2)

        # Match features
        knn = self._bf.knnMatch(des1, des2, k=2)
        good = [m for m, n in knn if m.distance < 0.75 * n.distance]
        
        if len(good) < self.cfg.min_inlier_count:
            return None, None, 0, None, None, None

        # Extract matched points and scale back to original resolution
        pts1 = np.array([kp1[m.queryIdx].pt for m in good], dtype=np.float32) / self._detection_scale
        pts2 = np.array([kp2[m.trainIdx].pt for m in good], dtype=np.float32) / self._detection_scale

        # Check for identical frames
        if np.allclose(gray1, gray2):
            return None, None, 0, None, None, None

        # Find essential matrix with lower RANSAC threshold for sub-pixel motion
        E, maskE = cv2.findEssentialMat(
            pts1, pts2,
            cameraMatrix=self._K,
            method=cv2.RANSAC,
            prob=0.9999,  # Higher probability
            threshold=0.5,  # Lower threshold for sub-pixel motion
        )
        
        if E is None or E.shape != (3, 3):
            return None, None, 0, None, None, None

        # Recover pose
        try:
            _, R, t, maskPose = cv2.recoverPose(
                E, pts1, pts2,
                cameraMatrix=self._K,
                mask=maskE,
            )
        except cv2.error:
            return None, None, 0, None, None, None

        # Get inlier points using pose mask
        inlier_mask = (maskPose.ravel() > 0)
        inliers = int(inlier_mask.sum())
        
        if inliers < self.cfg.min_inlier_count:
            return None, None, 0, None, None, None

        # Return inlier points for quality checks
        pts1_in = pts1[inlier_mask]
        pts2_in = pts2[inlier_mask]

        return R, t, inliers, pts1_in, pts2_in, E

    def _log_pose_estimation(self, inliers: int, frame_gap: int, pts1: Optional[np.ndarray] = None, pts2: Optional[np.ndarray] = None, E: Optional[np.ndarray] = None, scale: float = 0.0, rotation_only: bool = False):
        """Log concise pose estimation results with ground truth comparison and quality metrics."""
        if self._last_good_pose is None:
            return
            
        R_global, t_global = self._extract_SE3_components(self._T_wc)
        euler_global = self._rotation_matrix_to_euler_degrees(R_global)
        
        # Compute quality metrics if data available
        quality_info = ""
        if pts1 is not None and pts2 is not None and E is not None:
            # Compute both raw and normalized disparity stats
            raw_disp_stats = self._compute_raw_disparity_stats(pts1, pts2)
            parallax = self._compute_parallax(pts1, pts2, normalized=True)
            epipolar_error = self._compute_epipolar_error(pts1, pts2, E)
            U, S, Vt = np.linalg.svd(E)
            svd_ratio = float(S[1] / S[0]) if S[0] > 0 else 0.0
            
            # Determine state code
            if rotation_only:
                state_code = "R"  # Rotation-only
            elif raw_disp_stats['p95'] < 0.1:
                state_code = "L"  # Lost/texture
            elif raw_disp_stats['p95'] < 0.25:
                state_code = "Tμ"  # Micro-baseline translation
            else:
                state_code = "T"  # Normal translation
                
            quality_info = f" | State={state_code} | RawP95={raw_disp_stats['p95']:.2f}px | UsedP95={raw_disp_stats['p95']:.2f}px | ParallaxN={parallax:.3f} | Epipolar={epipolar_error:.2f}px | SVD={svd_ratio:.3f} | Scale={scale:.3f}m"
        
        # Get ground truth comparison
        if self.cfg.enable_gt_comparison:
            T_gt = self._get_ground_truth_pose()
            if T_gt is not None:
                R_gt, t_gt = self._extract_SE3_components(T_gt)
                euler_gt = self._rotation_matrix_to_euler_degrees(R_gt)
                
                # Compare rotation
                R_diff = R_global @ R_gt.T
                euler_diff = self._rotation_matrix_to_euler_degrees(R_diff)
                rot_error = np.linalg.norm(euler_diff)
                
                # Compare translation
                pos_error = np.linalg.norm(t_global - t_gt)
                
                # Get feature counts for diagnostics
                feat_prev = self._feat_prev
                feat_curr = self._feat_curr
                
                static_info = " [STATIC]" if self._is_static_mode else ""
                if rotation_only:
                    print(f"[POSE] Frame {self._frame_idx} | FeatPrev={feat_prev} FeatCurr={feat_curr} | Inliers: {inliers} | GT: RPY=({float(euler_gt[0]):.1f},{float(euler_gt[1]):.1f},{float(euler_gt[2]):.1f}) pos=({float(t_gt[0]):.2f},{float(t_gt[1]):.2f},{float(t_gt[2]):.2f}) | Est: RPY=({float(euler_global[0]):.1f},{float(euler_global[1]):.1f},{float(euler_global[2]):.1f}) pos=({float(t_global[0]):.2f},{float(t_global[1]):.2f},{float(t_global[2]):.2f}) | Errors: Rot={float(rot_error):.1f}° Pos={float(pos_error):.3f}m{static_info}{quality_info} | Rotation-only")
                else:
                    print(f"[POSE] Frame {self._frame_idx} | FeatPrev={feat_prev} FeatCurr={feat_curr} | Inliers: {inliers} | GT: RPY=({float(euler_gt[0]):.1f},{float(euler_gt[1]):.1f},{float(euler_gt[2]):.1f}) pos=({float(t_gt[0]):.2f},{float(t_gt[1]):.2f},{float(t_gt[2]):.2f}) | Est: RPY=({float(euler_global[0]):.1f},{float(euler_global[1]):.1f},{float(euler_global[2]):.1f}) pos=({float(t_global[0]):.2f},{float(t_global[1]):.2f},{float(t_global[2]):.2f}) | Errors: Rot={float(rot_error):.1f}° Pos={float(pos_error):.3f}m{static_info}{quality_info}")
            else:
                feat_prev = self._feat_prev
                feat_curr = self._feat_curr
                if rotation_only:
                    print(f"[POSE] Frame {self._frame_idx} | FeatPrev={feat_prev} FeatCurr={feat_curr} | Inliers: {inliers} | Est: RPY=({float(euler_global[0]):.1f},{float(euler_global[1]):.1f},{float(euler_global[2]):.1f}) pos=({float(t_global[0]):.2f},{float(t_global[1]):.2f},{float(t_global[2]):.2f}) | No GT available{quality_info} | Rotation-only")
                else:
                    print(f"[POSE] Frame {self._frame_idx} | FeatPrev={feat_prev} FeatCurr={feat_curr} | Inliers: {inliers} | Est: RPY=({float(euler_global[0]):.1f},{float(euler_global[1]):.1f},{float(euler_global[2]):.1f}) pos=({float(t_global[0]):.2f},{float(t_global[1]):.2f},{float(t_global[2]):.2f}) | No GT available{quality_info}")
        else:
            feat_prev = self._feat_prev
            feat_curr = self._feat_curr
            if rotation_only:
                print(f"[POSE] Frame {self._frame_idx} | FeatPrev={feat_prev} FeatCurr={feat_curr} | Inliers: {inliers} | Est: RPY=({float(euler_global[0]):.1f},{float(euler_global[1]):.1f},{float(euler_global[2]):.1f}) pos=({float(t_global[0]):.2f},{float(t_global[1]):.2f},{float(t_global[2]):.2f}){quality_info} | Rotation-only")
            else:
                print(f"[POSE] Frame {self._frame_idx} | FeatPrev={feat_prev} FeatCurr={feat_curr} | Inliers: {inliers} | Est: RPY=({float(euler_global[0]):.1f},{float(euler_global[1]):.1f},{float(euler_global[2]):.1f}) pos=({float(t_global[0]):.2f},{float(t_global[1]):.2f},{float(t_global[2]):.2f}){quality_info}")

    def _pre_physics_step(self, action):
        Task._pre_physics_step(self, action)

    # ------------------------------------------------------------------ #
    # Debug: tiny world-yaw increment to onboard camera every step.      #
    # ------------------------------------------------------------------ #
    def _add_translational_motion(self) -> bool:
        """Add controlled translational motion to create proper baseline."""
        if not hasattr(self, "camera_onboard"):
            return False
            
        self._motion_step_counter += 1
        
        # Only apply motion every N steps
        if self._motion_step_counter % self.cfg.motion_frequency != 0:
            return False
            
        try:
            cam = self.camera_onboard
            pos_w = cam.pos_w.clone()
            quat_w = cam.quat_w.clone()
            
            # Create sinusoidal motion in X and Y directions
            phase = (self._motion_step_counter / self.cfg.motion_frequency) * 2 * np.pi
            dx = self.cfg.motion_amplitude_m * np.sin(phase)
            dy = self.cfg.motion_amplitude_m * np.cos(phase * 0.7)  # Different frequency for variety
            
            # Apply translation
            new_pos = pos_w.clone()
            new_pos[0, 0] += dx  # X translation
            new_pos[0, 1] += dy  # Y translation
            
            cam.set_world_pose(new_pos, quat_w)
            
            print(f"[MOTION] Applied translation: dx={dx:.3f}m, dy={dy:.3f}m at step {self._motion_step_counter}")
            return True
            
        except Exception as e:
            print(f"[MOTION] Error applying translational motion: {e}")
            return False

    def _jitter_camera_yaw(self, delta_rad: float):
        if not hasattr(self, "camera_onboard"):
            return
        cam = self.camera_onboard
        pos_w = cam.pos_w.clone()
        qw, qx, qy, qz = cam.quat_w.cpu().numpy()
        half = delta_rad * 0.5
        s = np.sin(half)
        c = np.cos(half)
        dq = np.array([c, 0.0, 0.0, s], dtype=np.float64)  # (w,x,y,z)
        w0, x0, y0, z0 = dq
        w1, x1, y1, z1 = qw, qx, qy, qz
        q_new = np.array([
            w0*w1 - x0*x1 - y0*y1 - z0*z1,
            w0*x1 + x0*w1 + y0*z1 - z0*y1,
            w0*y1 - x0*z1 + y0*w1 + z0*x1,
            w0*z1 + x0*y1 - y0*x1 + z0*w1
        ], dtype=np.float64)
        cam.set_world_pose(pos_w, torch.tensor(q_new, device=self.device, dtype=torch.float32))

    @staticmethod
    def _rotation_matrix_to_euler_degrees(R: np.ndarray) -> np.ndarray:
        sy = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
        singular = sy < 1e-6
        if not singular:
            x = np.arctan2(R[2, 1], R[2, 2])
            y = np.arctan2(-R[2, 0], sy)
            z = np.arctan2(R[1, 0], R[0, 0])
        else:
            x = np.arctan2(-R[1, 2], R[1, 1])
            y = np.arctan2(-R[2, 0], sy)
            z = 0.0
        return np.array([np.degrees(x), np.degrees(y), np.degrees(z)])
