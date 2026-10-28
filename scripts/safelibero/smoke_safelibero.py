"""SafeLIBERO smoke: build one scene, settle, find the active obstacle."""
import json, sys
import numpy as np
from libero.libero import benchmark, get_libero_path
from libero.libero.envs import OffScreenRenderEnv
import pathlib

suite_name, level, task_id, ep = sys.argv[1], sys.argv[2], int(sys.argv[3]), int(sys.argv[4])
suite = benchmark.get_benchmark_dict()[suite_name](safety_level=level)
task = suite.get_task(task_id)
import torch
init_path = pathlib.Path(get_libero_path("init_states")) / task.problem_folder / task.init_states_file.replace(".pruned_init", f"_level_{level}.pruned_init")
inits = np.asarray(torch.load(init_path, weights_only=False))
bddl = pathlib.Path(get_libero_path("bddl_files")) / task.problem_folder / task.bddl_file
env = OffScreenRenderEnv(bddl_file_name=str(bddl), camera_heights=256, camera_widths=256, camera_depths=True)
env.seed(7)
env.reset()
obs = env.set_init_state(inits[ep])
for _ in range(10):
    obs, r, d, info = env.step([0.0] * 6 + [-1.0])
joint_names = list(env.env.sim.model.joint_names)
obstacles = [n.replace("_joint0", "") for n in joint_names if "obstacle" in n]
active = [n for n in obstacles if obs[f"{n}_pos"][2] > 0 and -0.5 < obs[f"{n}_pos"][0] < 0.5 and -0.5 < obs[f"{n}_pos"][1] < 0.5]
print(json.dumps({
    "suite": suite_name, "level": level, "task": task_id, "episode": ep,
    "n_tasks": suite.n_tasks, "n_inits": len(inits), "language": task.language,
    "bddl": str(bddl), "obstacles": obstacles, "active": active,
    "active_pos": [list(map(float, obs[f"{n}_pos"])) for n in active],
    "eef": list(map(float, obs["robot0_eef_pos"])),
    "keys_depth": [k for k in obs if "depth" in k],
    "success": bool(env.check_success()),
}))
env.close()
