import torch
import time

from srb.tasks.mobile.orbital_evasion.task_visual import VisualTask, VisualTaskCfg

def main():
    cfg = VisualTaskCfg()
    env = VisualTask(cfg)
    obs = env.reset()
    step = 0
    print("[ORBIT_RUNNER] Environment reset. Starting orbit...")
    while True:
        # Pass a dummy zero action; internal controller will set real action
        action = torch.zeros(env.action_space.shape, device=env.unwrapped.device)
        obs, reward, terminated, truncated, info = env.step(action)
        print(f"[ORBIT_RUNNER] Step {step} completed.")
        step += 1
        # Optional: sleep to slow down loop if needed
        # time.sleep(0.01)
        if terminated or truncated:
            print("[ORBIT_RUNNER] Episode ended. Resetting...")
            obs = env.reset()
            step = 0

if __name__ == "__main__":
    main() 