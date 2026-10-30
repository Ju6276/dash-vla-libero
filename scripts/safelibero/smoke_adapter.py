"""Drive SafeLiberoRpcEnv directly: reset, push the arm into the obstacle region, read the trace."""
import json, sys
import numpy as np
from robots.libero.scene_env import SafeLiberoRpcEnv, safelibero_task_identity

suite, task, seed = sys.argv[1], int(sys.argv[2]), int(sys.argv[3])
env = SafeLiberoRpcEnv(suite, task, seed, 10000)
obs, _ = env.reset()
print("obs keys", sorted(obs), obs["main_images"].shape, obs["states"].shape, obs["task_descriptions"])
start = np.asarray(env._raw["robot0_eef_pos"])
target = np.asarray(env.obstacle_start) + np.array([0, 0, 0.02])
for i in range(250):
    eef = np.asarray(env._raw["robot0_eef_pos"])
    a = np.zeros(7, dtype=np.float32)
    a[:3] = np.clip((target - eef) / 0.05, -1, 1)
    a[6] = -1
    env.step(a)
    if env.collided:
        break
t = env.safety_trace()
print(json.dumps({k: t[k] for k in ("obstacle", "obstacle_start", "collided", "collide_step", "max_displacement_m", "control_steps", "check_success", "audited_positive_steps")}))
print("first audits", json.dumps(t["step_audits"][:3]))
snap = env.scene_snapshot()
print("snapshot keys", sorted(snap), snap["obstacle"])
env.close()
