"""Export SafeLIBERO initial scenes as the harness sees them, plus ground truth.

For each cell (suite:task:seed) writes OUT/<cell>/:
  image.png   agentview 1024x1024, the frame ground_box segments
  world.npz   per-pixel world xyz for that image (as agentview_world_high.npz)
  scene.json  task language, planner-visible object names, the active obstacle
              under its planner-visible name, and ground-truth world boxes of
              every object under planner-visible names (evaluation only)

Usage (LIBERO-Safety venv, same env vars as run_safelibero_harness.sh):
  export_scenes.py --out DIR suite:task:seed ...
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import imageio.v2 as imageio
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from scene_geometry import object_box  # noqa: E402

from robots.libero.env_server import make_env  # noqa: E402
from robots.libero.tools import _metric_depth, _world_from_depth  # noqa: E402


def planner_names(env) -> dict[str, str]:
    """Benchmark object name -> name in the planner's observation (None if hidden)."""
    names = [k[:-4] for k in env._raw if k.endswith("_pos") and "robot0" not in k and "to_robot" not in k]
    view = env._planner_view({f"{n}_pos": n for n in names})
    shown = {orig: key[:-4] for key, orig in view.items()}
    return {n: shown.get(n) for n in names}


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("cells", nargs="+")
    args = parser.parse_args()
    for cell in args.cells:
        out = args.out / cell.replace(":", "_")
        if (out / "scene.json").is_file():
            continue
        suite, task, seed = cell.split(":")
        env = make_env(int(task), int(seed), suite, 300)
        rgb, depth = env.render_camera("agentview", 1024, 1024, True)
        meta = env.get_camera_meta("agentview", 1024, 1024)
        world = _world_from_depth(_metric_depth(depth, meta)[::-1], meta).astype(np.float16)
        mapping = planner_names(env)
        domain = env._sim.env
        boxes = {
            mapping[name]: object_box(domain.sim, body)
            for name, body in domain.obj_body_id.items()
            if mapping.get(name)
        }
        out.mkdir(parents=True, exist_ok=True)
        imageio.imwrite(out / "image.png", np.asarray(rgb)[::-1])
        np.savez_compressed(out / "world.npz", world=world)
        scene = {
            "cell": cell,
            "language": env.language,
            "objects": sorted(n for n in mapping.values() if n),
            "obstacle": mapping.get(env.obstacle_name),
            "eef_start": [float(v) for v in env._raw["robot0_eef_pos"]],
            "gt_boxes": boxes,
        }
        (out / "scene.json").write_text(json.dumps(scene, indent=1))
        env.close()
        print(cell, scene["obstacle"], flush=True)


if __name__ == "__main__":
    main()
