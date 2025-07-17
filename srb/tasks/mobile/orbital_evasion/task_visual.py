# tasks/mobile/orbital_evasion/task_visual.py

from typing import Dict, Sequence, Optional
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
    # Debug / tuning params (Hydra overrides OK)
    debug_cam_jitter_rad: float = 0.0    # tiny per-step yaw nudge (rad)
    min_inlier_count:     int   = 5      # min RANSAC inliers to accept pose
    ransac_thresh_px:     float = 1.5    # RANSAC pixel thresh

    def __post_init__(self):
        TaskCfg.__post_init__(self)
        OrbitalEnvVisualExtCfg.wrap(self, env_cfg=self)


class VisualTask(VisualExt, Task):
    cfg: VisualTaskCfg

    def __init__(self, cfg: VisualTaskCfg, **kwargs):
        print("[VISUAL] Initializing VisualTask...")
        Task.__init__(self, cfg, **kwargs)
        VisualExt.__init__(self, cfg, **kwargs)

        # ORB & matcher (optimized for small images)
        self._orb = cv2.ORB_create(
            nfeatures=500,      # Reduced for small images
            scaleFactor=1.1,    # Smaller scale factor
            nlevels=8,          # More levels for small images
            edgeThreshold=15,   # Smaller edge threshold
            firstLevel=0,       # Start from original scale
            WTA_K=2,            # 2-point random sampling
            patchSize=31,       # Smaller patch size
            fastThreshold=10    # Lower threshold for more features
        )
        self._bf  = cv2.BFMatcher(cv2.NORM_HAMMING)

        # Camera intrinsics (SRB default onboard cam 640x480 @ 60° HFOV)
        w, h = 640, 480
        self._pp    = (w / 2.0, h / 2.0)
        self._focal = (w / 2.0) / np.tan(np.deg2rad(60.0 / 2.0))
        self._K = np.array([[self._focal, 0.0, self._pp[0]],
                            [0.0, self._focal, self._pp[1]],
                            [0.0,        0.0,          1.0]], dtype=np.float64)

        self._prev_frame: Optional[np.ndarray] = None    # RGB
        self._prev_gray:  Optional[np.ndarray] = None    # gray
        self._step_counter = 0
        self._last_good_pose = None  # (R, t, inliers)

        print(f"[VISUAL] intrinsics: f={self._focal:.2f}px, pp={self._pp}")

    # ------------------------------------------------------------------ #
    # Merge Task + VisualExt observations                                #
    # ------------------------------------------------------------------ #
    def _get_observations(self) -> Dict[str, torch.Tensor]:
        return {**Task._get_observations(self), **VisualExt._get_observations(self)}

    # ------------------------------------------------------------------ #
    # Extract step return - called every step by SRB framework           #
    # ------------------------------------------------------------------ #
    def extract_step_return(self) -> StepReturn:
        # Run pose estimation for environment 0
        print(f"[DEBUG] extract_step_return called, step={self._step_counter}")
        
        # Optional tiny yaw jitter to create parallax (before image capture)
        if self.cfg.debug_cam_jitter_rad != 0.0:
            try:
                self._jitter_camera_yaw(self.cfg.debug_cam_jitter_rad)
                print(f"[DEBUG] Applied camera jitter: {self.cfg.debug_cam_jitter_rad} rad")
            except Exception as e:
                print(f"[VISUAL] camera jitter failed: {e}")

        try:
            # Grab current image directly from VisualExt (avoid base class _get_observations)
            visual_obs = VisualExt._get_observations(self)
            print(f"[DEBUG] Got visual observations: {list(visual_obs.keys())}")
            
            if "image_onboard" not in visual_obs:
                print("[DEBUG] No image_onboard in visual observations!")
            else:
                img_t = visual_obs["image_onboard"][0]  # [C,H,W] float[0..1]
                print(f"[DEBUG] Image tensor shape: {img_t.shape}")
                
                # Handle different image formats
                if img_t.shape[2] == 4:  # RGBA (H, W, 4)
                    img_t = img_t[:, :, :3]  # Take only RGB channels
                    print(f"[DEBUG] Converted RGBA to RGB: {img_t.shape}")
                elif img_t.shape[2] == 1:  # Grayscale (H, W, 1)
                    img_t = img_t.repeat(1, 1, 3)  # Convert to RGB
                    print(f"[DEBUG] Converted grayscale to RGB: {img_t.shape}")
                
                # Convert to numpy (tensor is already in HWC format)
                img = (img_t.cpu().numpy() * 255).astype(np.uint8)
                print(f"[DEBUG] Final image shape: {img.shape}, dtype: {img.dtype}")
                
                gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
                print(f"[DEBUG] Converted to gray: {gray.shape}")
                
                # Debug: Check image statistics
                print(f"[DEBUG] Image stats - min: {gray.min()}, max: {gray.max()}, mean: {gray.mean():.1f}")
                print(f"[DEBUG] Image unique values: {len(np.unique(gray))}")

                # Run pose when we have a prior
                if self._prev_gray is not None:
                    print(f"[DEBUG] Running 5-point algorithm...")
                    self._run_five_point(self._prev_gray, gray, self._prev_frame, img)
                else:
                    print("[POSE] waiting for 2nd frame...")

                # Store current frame for next iteration
                self._prev_gray = gray
                self._prev_frame = img
                self._step_counter += 1
                
        except Exception as e:
            print(f"[DEBUG] Error in extract_step_return: {e}")
            import traceback
            traceback.print_exc()
        
        # Call parent extract_step_return to get the actual step return
        return Task.extract_step_return(self)

    # ------------------------------------------------------------------ #
    # Debug: tiny world-yaw increment to onboard camera every step.      #
    # ------------------------------------------------------------------ #
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

    # ------------------------------------------------------------------ #
    # 5-point pose w/ RANSAC + diagnostics                               #
    # ------------------------------------------------------------------ #
    def _run_five_point(self, gray1, gray2, img1, img2):
        kp1, des1 = self._orb.detectAndCompute(gray1, None)
        kp2, des2 = self._orb.detectAndCompute(gray2, None)

        if des1 is None or des2 is None or len(kp1) < 5 or len(kp2) < 5:
            print(f"[POSE] insufficient features: f1={len(kp1) if kp1 else 0}, f2={len(kp2) if kp2 else 0}")
            return

        knn = self._bf.knnMatch(des1, des2, k=2)
        good = [m for m,n in knn if m.distance < 0.75 * n.distance]
        if len(good) < self.cfg.min_inlier_count:
            print(f"[POSE] insufficient good matches after ratio: {len(good)}")
            return

        pts1 = np.float32([kp1[m.queryIdx].pt for m in good])
        pts2 = np.float32([kp2[m.trainIdx].pt for m in good])

        if np.allclose(gray1, gray2):
            print("[POSE] frames identical (no baseline) -> skipping E.")
            return

        E, maskE = cv2.findEssentialMat(
            pts1, pts2,
            cameraMatrix=self._K,
            method=cv2.RANSAC,
            prob=0.999,
            threshold=self.cfg.ransac_thresh_px,
        )
        if E is None:
            print("[POSE] Essential matrix failed (degenerate / all outliers).")
            return

        inliers = int(maskE.sum()) if maskE is not None else 0
        if inliers < self.cfg.min_inlier_count:
            print(f"[POSE] too few inliers from RANSAC: {inliers}")
            return

        _, R, t, maskPose = cv2.recoverPose(
            E, pts1, pts2,
            cameraMatrix=self._K,
            mask=maskE,
        )

        euler = self._rotation_matrix_to_euler_degrees(R)
        sp_pos = self._robot.data.root_pos_w[0].cpu().numpy()
        ast_pos = self._objs.data.object_com_pos_w[0, 0].cpu().numpy()
        rel_dist = np.linalg.norm(ast_pos - sp_pos)

        self._last_good_pose = (R.copy(), t.copy(), int(maskPose.sum()))

        print("\n[POSE] ================= 5-POINT RESULT =================")
        print(f"[POSE] step={self._step_counter}  inliers={inliers}/{len(good)}")
        print(f"[POSE] RPY(deg)=({euler[0]:.2f}, {euler[1]:.2f}, {euler[2]:.2f})")
        print(f"[POSE] t_dir={t.ravel()}  |t|={np.linalg.norm(t):.4f} (scale-free)")
        print(f"[POSE] spacecraft@{sp_pos}  asteroid@{ast_pos}  dist={rel_dist:.3f} m")
        print("[POSE] ====================================================")

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
