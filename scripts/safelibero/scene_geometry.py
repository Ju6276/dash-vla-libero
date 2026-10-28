#!/usr/bin/env python3
"""World-frame collision boxes of every SafeLIBERO object after the settle steps.

The Harness traces record the end-effector path and the obstacle position but
not object extents; route_check.py needs those to replay a SafeHarness-style
check offline. Initial states are deterministic per (suite, task, seed), so
resetting the same cell reproduces the scene the planner saw.

Usage: scene_geometry.py --out geometry.json safelibero_goal_I:0:1 ...
"""

from __future__ import annotations

import argparse
import itertools
import json
from pathlib import Path

import numpy as np

from robots.libero.env_server import make_env


def _subtree(model, root: int) -> set[int]:
    bodies = {root}
    for body in range(model.nbody):
        parent = body
        while parent not in bodies and parent != 0:
            parent = model.body_parentid[parent]
        if parent in bodies:
            bodies.add(body)
    return bodies


def object_box(sim, body_id: int) -> dict:
    """Axis-aligned world box over the body's collision geoms (contype or conaffinity set)."""
    model, data = sim.model._model, sim.data._data
    bodies = _subtree(model, body_id)
    corners = []
    for geom in range(model.ngeom):
        if model.geom_bodyid[geom] not in bodies:
            continue
        if model.geom_contype[geom] == 0 and model.geom_conaffinity[geom] == 0:
            continue
        center, half = model.geom_aabb[geom, :3], model.geom_aabb[geom, 3:]
        rot = data.geom_xmat[geom].reshape(3, 3)
        for signs in itertools.product((-1, 1), repeat=3):
            corners.append(data.geom_xpos[geom] + rot @ (center + np.asarray(signs) * half))
    if not corners:
        return {}
    corners = np.asarray(corners)
    return {"min": corners.min(0).round(4).tolist(), "max": corners.max(0).round(4).tolist()}


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("cells", nargs="+", help="suite:task:seed")
    args = parser.parse_args()
    result = json.loads(args.out.read_text()) if args.out.exists() else {}
    for cell in args.cells:
        if cell in result:
            continue
        suite, task, seed = cell.split(":")
        env = make_env(int(task), int(seed), suite, 300)
        domain = env._sim.env
        result[cell] = {
            "obstacle": env.obstacle_name,
            "eef_start": [float(v) for v in env._raw["robot0_eef_pos"]] if env._raw else None,
            "objects": {
                name: object_box(domain.sim, body)
                for name, body in domain.obj_body_id.items()
            },
        }
        env.close()
        print(cell, env.obstacle_name, result[cell]["objects"].get(env.obstacle_name), flush=True)
        args.out.write_text(json.dumps(result, indent=1))


if __name__ == "__main__":
    main()
