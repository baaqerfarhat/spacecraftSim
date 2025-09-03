"""
ROS-based RL training to make the spacecraft follow a circular orbit around the asteroid.

This script defines a Gymnasium environment that:
- Publishes thruster commands (8-d continuous action) over ROS2.
- Reads spacecraft and asteroid poses from TF to compute observations and rewards.
- Trains a PPO policy (stable-baselines3) to follow a circular path of a chosen diameter/speed.

Run inside your Isaac/ROS container:
  python -m space_robotics_bench.rl.ros_orbit_following_train \
    --thrust-topic /srb/env0/robot/thrust \
    --world-frame world \
    --robot-frame robot \
    --center-frame apophis_unique \
    --diameter 100.0 \
    --speed 2.0 \
    --timesteps 1000000 \
    --model-out ./space_robotics_bench/logs/ppo_orbit

Notes:
- Uses TF only; no dependency on Odometry topics.
- Assumes the SRB ROS interface is active and broadcasting TF frames for `world` and `robot`.
"""

from __future__ import annotations

import argparse
import math
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Tuple, Optional

import numpy as np

try:
    import gymnasium as gym
    from gymnasium import spaces
except Exception as e:  # pragma: no cover
    raise RuntimeError("Please install gymnasium.") from e

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import MultiThreadedExecutor
    from std_msgs.msg import Float32MultiArray
    import tf2_ros
    from geometry_msgs.msg import TransformStamped
except Exception as e:  # pragma: no cover
    raise RuntimeError("ROS2 Python packages not available in this environment.") from e


@dataclass
class OrbitConfig:
    thrust_topic: str = "/srb/env0/robot/thrust"
    world_frame: str = "world"
    robot_frame: str = "robot"
    center_frame: str = "apophis_unique"
    diameter_m: float = 100.0
    tangential_speed_mps: float = 2.0
    duty_cap: float = 0.40
    dt_s: float = 0.05
    episode_seconds: float = 60.0
    far_reset_radius_factor: float = 3.0
    z_penalty_weight: float = 0.2
    radial_error_weight: float = 1.0
    tangential_speed_weight: float = 0.5
    radial_speed_weight: float = 0.1
    control_penalty_weight: float = 0.01


class _ROSNode(Node):
    def __init__(self, cfg: OrbitConfig):
        super().__init__("ros_orbit_following_env", namespace="srb")
        self.cfg = cfg
        self._pub_thrusters = self.create_publisher(Float32MultiArray, cfg.thrust_topic, 10)
        self._tf_buffer = tf2_ros.Buffer(cache_time=rclpy.time.Duration(seconds=5))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

    def publish_thrusters(self, u: np.ndarray):
        msg = Float32MultiArray()
        msg.data = u.astype(np.float32).tolist()
        self._pub_thrusters.publish(msg)

    def lookup_tf(self, target_frame: str, source_frame: str) -> Optional[TransformStamped]:
        try:
            return self._tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=rclpy.time.Duration(seconds=0.1),
            )
        except Exception:
            return None


class ROSOrbitEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(self, cfg: OrbitConfig):
        super().__init__()
        self.cfg = cfg
        if not rclpy.utilities.ok():
            rclpy.init()
        self._node = _ROSNode(cfg)
        self._executor = MultiThreadedExecutor(num_threads=2)
        self._executor.add_node(self._node)
        self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._spin_thread.start()

        self._radius = max(1e-3, cfg.diameter_m * 0.5)
        self._omega = cfg.tangential_speed_mps / self._radius

        self._last_pos_w: Optional[np.ndarray] = None
        self._last_t: Optional[float] = None
        self._t0: float = 0.0

        high_obs = np.array([
            1e3, 1e3,  # rel_pos_xy
            1e2, 1e2,  # rel_vel_xy
            1e3,       # rel_pos_z
            1e2,       # rel_vel_z
            1e3,       # radial_error
            1e2, 1e2,  # tangential_speed, radial_speed
            1.0, 1.0,  # cos(theta), sin(theta)
        ], dtype=np.float32)
        self.observation_space = spaces.Box(low=-high_obs, high=high_obs, dtype=np.float32)
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(8,), dtype=np.float32)

    def _get_positions(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        t_robot = self._node.lookup_tf(self.cfg.world_frame, self.cfg.robot_frame)
        t_center = self._node.lookup_tf(self.cfg.world_frame, self.cfg.center_frame)
        p_robot = None
        p_center = None
        if t_robot is not None:
            p_robot = np.array([
                t_robot.transform.translation.x,
                t_robot.transform.translation.y,
                t_robot.transform.translation.z,
            ], dtype=np.float64)
        if t_center is not None:
            p_center = np.array([
                t_center.transform.translation.x,
                t_center.transform.translation.y,
                t_center.transform.translation.z,
            ], dtype=np.float64)
        return p_robot, p_center

    def _compute_obs_and_reward(self, u: np.ndarray | None) -> Tuple[np.ndarray, float, dict]:
        p_robot, p_center = self._get_positions()
        if p_robot is None or p_center is None:
            obs = np.zeros(self.observation_space.shape, dtype=np.float32)
            return obs, 0.0, {"valid": False}

        now = time.time()
        dt = self.cfg.dt_s if self._last_t is None else max(1e-3, now - self._last_t)
        v_robot = (
            np.zeros(3, dtype=np.float64)
            if self._last_pos_w is None
            else (p_robot - self._last_pos_w) / dt
        )

        rel = p_robot - p_center
        rel_xy = rel[:2]
        r = float(np.linalg.norm(rel_xy) + 1e-9)
        r_hat = rel_xy / r
        t_hat = np.array([-r_hat[1], r_hat[0]], dtype=np.float64)
        theta = math.atan2(rel_xy[1], rel_xy[0])

        v_xy = v_robot[:2]
        v_t = float(np.dot(v_xy, t_hat))
        v_r = float(np.dot(v_xy, r_hat))
        radial_err = r - self._radius

        reward = 0.0
        reward -= self.cfg.radial_error_weight * (radial_err ** 2)
        reward -= self.cfg.tangential_speed_weight * ((v_t - self.cfg.tangential_speed_mps) ** 2)
        reward -= self.cfg.radial_speed_weight * (v_r ** 2)
        reward -= self.cfg.z_penalty_weight * (rel[2] ** 2)
        if u is not None:
            reward -= self.cfg.control_penalty_weight * float(np.mean(np.abs(u)))

        obs = np.array([
            rel_xy[0], rel_xy[1],
            v_xy[0], v_xy[1],
            rel[2], v_robot[2],
            radial_err,
            v_t, v_r,
            math.cos(theta), math.sin(theta),
        ], dtype=np.float32)

        self._last_pos_w = p_robot
        self._last_t = now

        info = {
            "valid": True,
            "radius": r,
            "radial_err": radial_err,
            "v_t": v_t,
            "v_r": v_r,
        }
        return obs, float(reward), info

    def step(self, action: np.ndarray):
        u = np.clip(action.astype(np.float32), -1.0, 1.0)
        duty = (self.cfg.duty_cap * u).astype(np.float32)
        self._node.publish_thrusters(duty)

        obs, reward, info = self._compute_obs_and_reward(u)

        elapsed = time.time() - self._t0
        terminated = False
        truncated = elapsed >= self.cfg.episode_seconds

        if info.get("valid", False):
            r = info["radius"]
            if r > self.cfg.far_reset_radius_factor * self._radius:
                truncated = True

        time.sleep(self.cfg.dt_s)
        return obs, reward, terminated, truncated, info

    def reset(self, *, seed: int | None = None, options: dict | None = None):
        super().reset(seed=seed)
        self._node.publish_thrusters(np.zeros(8, dtype=np.float32))
        time.sleep(0.1)
        self._last_pos_w = None
        self._last_t = None
        self._t0 = time.time()

        obs, _, info = self._compute_obs_and_reward(None)
        if not info.get("valid", False):
            # Give TF a moment to become available at episode start
            for _ in range(20):
                time.sleep(0.05)
                obs, _, info = self._compute_obs_and_reward(None)
                if info.get("valid", False):
                    break
        return obs, info

    def close(self):
        try:
            self._node.publish_thrusters(np.zeros(8, dtype=np.float32))
        except Exception:
            pass
        try:
            self._executor.shutdown()
        except Exception:
            pass
        try:
            if rclpy.utilities.ok():
                rclpy.shutdown()
        except Exception:
            pass


def _train(cfg: OrbitConfig, timesteps: int, out_dir: Path, policy: str = "MlpPolicy"):
    try:
        from stable_baselines3 import PPO
    except Exception as e:
        raise RuntimeError("Please install stable-baselines3 inside your container.") from e

    env = ROSOrbitEnv(cfg)
    model = PPO(policy, env, verbose=1, tensorboard_log=str(out_dir))
    model.learn(total_timesteps=int(timesteps))
    out_dir.mkdir(parents=True, exist_ok=True)
    model.save(out_dir / "ppo_orbit_following.zip")
    env.close()


def main():
    parser = argparse.ArgumentParser(description="Train RL policy to orbit asteroid using thrusters over ROS2")
    parser.add_argument("--thrust-topic", type=str, default="/srb/env0/robot/thrust")
    parser.add_argument("--world-frame", type=str, default="world")
    parser.add_argument("--robot-frame", type=str, default="robot")
    parser.add_argument("--center-frame", type=str, default="apophis_unique")
    parser.add_argument("--diameter", type=float, default=100.0)
    parser.add_argument("--speed", type=float, default=2.0)
    parser.add_argument("--duty-cap", type=float, default=0.40)
    parser.add_argument("--dt", type=float, default=0.05)
    parser.add_argument("--episode-seconds", type=float, default=60.0)
    parser.add_argument("--timesteps", type=int, default=1_000_000)
    parser.add_argument("--model-out", type=str, default="./space_robotics_bench/logs/ppo_orbit")
    args = parser.parse_args()

    cfg = OrbitConfig(
        thrust_topic=args.thrust_topic,
        world_frame=args.world_frame,
        robot_frame=args.robot_frame,
        center_frame=args.center_frame,
        diameter_m=args.diameter,
        tangential_speed_mps=args.speed,
        duty_cap=args.duty_cap,
        dt_s=args.dt,
        episode_seconds=args.episode_seconds,
    )
    _train(cfg, args.timesteps, Path(args.model_out))


if __name__ == "__main__":
    main()


