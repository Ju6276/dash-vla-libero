"""Offline check for orientation-aware clearance (DASH module 3).

On ground-truth geometry of the exported scenes (export_scenes.py), compare two
rules for the VLA hand-off poses a SafeHarness-style harness must reach: above
the pick target, and above the place location while holding the target.

  fixed   SafeHarness: the gripper point is >= 12 cm horizontally from the
          obstacle box or >= 5 cm above it.
  orient  some wrist yaw keeps the measured Panda hand footprint (rotated
          rectangle, +-4.4 x +-10.4 cm) >= 2 cm from the obstacle box over the
          whole vertical descent from the hand-off pose to the contact height,
          or the hand stays >= 5 cm above it.

Ground truth for both: does the hand at the robot's default wrist yaw intersect
the obstacle during that descent (what happens if nothing reorients the wrist)?

Obstacle sets: the SafeLIBERO obstacle only, and every object the task goal
does not involve (module 1's inventory, scored on ground-truth boxes).

Usage (LIBERO-Safety venv for the default yaw; geometry is pure numpy):
  clearance_rules.py SCENES_DIR --bddl-root DIR [--out summary.json]
"""

from __future__ import annotations

import argparse
import json
import math
import re
from pathlib import Path

import numpy as np

HORIZONTAL = 0.12
VERTICAL = 0.05
MARGIN = 0.02
HAND_HALF_X = 0.044
HAND_HALF_Y = 0.104
HAND_ABOVE_EEF = 0.12
GRIPPER_BELOW_EEF = 0.01
HANDOFF_ABOVE = 0.03
DEFAULT_YAW = math.pi / 2  # opening axis (EEF y) is world -y at the LIBERO start pose (probed in sim)
YAWS = np.deg2rad(np.arange(0, 180, 5))


def goal_pairs(bddl: str) -> list[tuple[str, str]]:
    goal = bddl[bddl.index("(:goal"):]
    return [(a, b) for _, a, b in re.findall(r"\((On|In)\s+(\S+)\s+(\S+?)\)", goal)]


def base(name: str) -> str:
    return re.sub(r"_\d+$", "", name)


def resolve(name: str, boxes: dict) -> str | None:
    """BDDL object (or region like plate_1 / basket_1_contain_region) -> scene object."""
    for key in boxes:
        if name == key or name.startswith(key + "_"):
            return key
    return None


def footprint(xy, yaw: float) -> np.ndarray:
    c, s = math.cos(yaw), math.sin(yaw)
    # yaw is the opening-axis heading: the long side (half_y) points along (c, s)
    long_axis, short_axis = np.array([c, s]), np.array([-s, c])
    return np.array([xy + a * HAND_HALF_Y * long_axis + b * HAND_HALF_X * short_axis
                     for a, b in ((1, 1), (1, -1), (-1, -1), (-1, 1))])


def _seg_dist(p, a, b) -> float:
    ab = b - a
    t = np.clip(np.dot(p - a, ab) / (np.dot(ab, ab) + 1e-12), 0, 1)
    return float(np.linalg.norm(p - (a + t * ab)))


def _overlap(poly_a, poly_b) -> bool:
    for poly in (poly_a, poly_b):
        for i in range(len(poly)):
            edge = poly[(i + 1) % len(poly)] - poly[i]
            axis = np.array([-edge[1], edge[0]])
            pa, pb = poly_a @ axis, poly_b @ axis
            if pa.max() < pb.min() or pb.max() < pa.min():
                return False
    return True


def poly_gap(poly_a, poly_b) -> float:
    """Distance between convex polygons, negative (-1) when they overlap."""
    if _overlap(poly_a, poly_b):
        return -1.0
    best = math.inf
    for p, q in ((poly_a, poly_b), (poly_b, poly_a)):
        for v in p:
            for i in range(len(q)):
                best = min(best, _seg_dist(v, q[i], q[(i + 1) % len(q)]))
    return best


def box_poly(box: dict) -> np.ndarray:
    (x0, y0), (x1, y1) = box["min"][:2], box["max"][:2]
    return np.array([[x0, y0], [x1, y0], [x1, y1], [x0, y1]])


def rect_point_distance(xy, box: dict) -> float:
    dx = max(box["min"][0] - xy[0], 0, xy[0] - box["max"][0])
    dy = max(box["min"][1] - xy[1], 0, xy[1] - box["max"][1])
    return math.hypot(dx, dy)


def fixed_ok(eef, obstacle: dict) -> bool:
    gripper_z = eef[2] - GRIPPER_BELOW_EEF
    return (rect_point_distance(eef[:2], obstacle) >= HORIZONTAL
            or gripper_z - obstacle["max"][2] >= VERTICAL)


def descent_gap(xy, z_low, z_high, yaw, obstacle: dict, held: dict | None) -> float:
    """Smallest horizontal footprint gap over the descent where heights overlap; inf if none do."""
    lo = z_low - GRIPPER_BELOW_EEF - (held_height(held) if held else 0.0)
    hi = z_high + HAND_ABOVE_EEF
    if obstacle["max"][2] + VERTICAL <= lo or obstacle["min"][2] >= hi:
        return math.inf
    gap = poly_gap(footprint(xy, yaw), box_poly(obstacle))
    if held:
        gap = min(gap, poly_gap(held_poly(xy, held), box_poly(obstacle)))
    return gap


def held_height(held: dict) -> float:
    return held["max"][2] - held["min"][2]


def held_poly(xy, held: dict) -> np.ndarray:
    hx = (held["max"][0] - held["min"][0]) / 2
    hy = (held["max"][1] - held["min"][1]) / 2
    return np.array([xy + [sx * hx, sy * hy] for sx, sy in ((1, 1), (1, -1), (-1, -1), (-1, 1))])


def hits(xy, z_low, z_high, yaw, obstacle, held) -> bool:
    """Ground truth: the hand (or held object) intersects the obstacle somewhere on the descent."""
    lo = z_low - GRIPPER_BELOW_EEF - (held_height(held) if held else 0.0)
    hi = z_high + HAND_ABOVE_EEF
    if obstacle["max"][2] <= lo or obstacle["min"][2] >= hi:
        return False
    return descent_gap(xy, z_low, z_high, yaw, obstacle, held) < 0


def evaluate(pose: dict, obstacles: dict) -> dict:
    xy, z_high, z_low, held = pose["xy"], pose["z_handoff"], pose["z_contact"], pose["held"]
    eef = np.array([*xy, z_high])
    fixed = all(fixed_ok(eef, box) for box in obstacles.values())
    feasible = [yaw for yaw in YAWS
                if all(descent_gap(xy, z_low, z_high, yaw, box, held) >= MARGIN for box in obstacles.values())]
    default_hits = [name for name, box in obstacles.items()
                    if hits(xy, z_low, z_high, DEFAULT_YAW, box, held)]
    return {
        "fixed_pass": fixed,
        "orient_pass": bool(feasible),
        "feasible_yaw_fraction": round(len(feasible) / len(YAWS), 3),
        "default_yaw_hits": default_hits,
    }


def poses(scene: dict, pairs) -> list[dict]:
    boxes = scene["gt_boxes"]
    out = []
    for obj, place in pairs:
        target, region = resolve(obj, boxes), resolve(place, boxes)
        if target is None:
            continue
        t = boxes[target]
        txy = (np.asarray(t["min"][:2]) + np.asarray(t["max"][:2])) / 2
        out.append({"phase": "pick", "object": target, "xy": txy, "held": None,
                    "z_handoff": t["max"][2] + HANDOFF_ABOVE, "z_contact": (t["min"][2] + t["max"][2]) / 2})
        if region is None:
            continue
        p = boxes[region]
        pxy = (np.asarray(p["min"][:2]) + np.asarray(p["max"][:2])) / 2
        held = {"min": [0, 0, 0], "max": np.subtract(t["max"], t["min"]).tolist()}
        out.append({"phase": "place", "object": region, "xy": pxy, "held": held,
                    "z_handoff": p["max"][2] + held_height(held) + HANDOFF_ABOVE,
                    "z_contact": p["max"][2] + held_height(held)})
    return out


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("scenes", type=Path)
    parser.add_argument("--bddl-root", type=Path, required=True)
    parser.add_argument("--out", type=Path)
    args = parser.parse_args()
    rows = []
    for scene_dir in sorted(p for p in args.scenes.iterdir() if (p / "scene.json").is_file()):
        scene = json.loads((scene_dir / "scene.json").read_text())
        suite = scene["cell"].split(":")[0]
        folder = re.sub(r"_(I|II)$", "", suite)
        (bddl,) = (args.bddl_root / folder).glob("*" + scene["language"].replace(" ", "_") + ".bddl")
        pairs = goal_pairs(bddl.read_text())
        task_objects = {resolve(n, scene["gt_boxes"]) for pair in pairs for n in pair}
        boxes = scene["gt_boxes"]
        sets = {
            "obstacle": {scene["obstacle"]: boxes[scene["obstacle"]]},
            "inventory": {n: b for n, b in boxes.items() if n not in task_objects and b},
        }
        for pose in poses(scene, pairs):
            for set_name, obstacles in sets.items():
                obstacles = {n: b for n, b in obstacles.items() if n != pose["object"]}
                row = {"cell": scene["cell"], "phase": pose["phase"], "object": pose["object"],
                       "set": set_name, **evaluate(pose, obstacles)}
                rows.append(row)
    summary = {}
    for set_name in ("obstacle", "inventory"):
        for phase in ("pick", "place", "all"):
            sel = [r for r in rows if r["set"] == set_name and (phase == "all" or r["phase"] == phase)]
            if not sel:
                continue
            n = len(sel)
            summary[f"{set_name}/{phase}"] = {
                "poses": n,
                "fixed_pass": sum(r["fixed_pass"] for r in sel),
                "orient_pass": sum(r["orient_pass"] for r in sel),
                "default_yaw_hits": sum(bool(r["default_yaw_hits"]) for r in sel),
                "fixed_pass_but_default_hits": sum(r["fixed_pass"] and bool(r["default_yaw_hits"]) for r in sel),
                "fixed_refuses_orient_admits": sum(not r["fixed_pass"] and r["orient_pass"] for r in sel),
                "both_refuse": sum(not r["fixed_pass"] and not r["orient_pass"] for r in sel),
            }
    print(json.dumps(summary, indent=1))
    if args.out:
        args.out.write_text(json.dumps({"summary": summary, "rows": rows}, indent=1))


if __name__ == "__main__":
    main()
