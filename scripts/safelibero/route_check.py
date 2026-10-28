#!/usr/bin/env python3
"""Offline estimate: which original-Harness collisions a SafeHarness-style check covers.

SafeHarness (arXiv 2609.20822) grounds objects as boxes, verifies every
analytic waypoint route against the obstacle box with a clearance margin
(the gripper and any held object), and invokes the frozen VLA only once the
gripper is within 3 cm of the target. We have no code for it, so this replays
that rule on the recorded B0 trajectories:

  analytic motion   covered if the straight start->end sweep of the gripper
                    box (plus held object) enters the obstacle box + clearance
  VLA call          not covered if the VLA started within 3 cm (horizontal) of
                    where it ended, i.e. SafeHarness would also have run it
                    there; otherwise "plausible": SafeHarness would have done
                    that travel analytically, and covered it if the sweep
                    is flagged
  release           covered if the release pose is inside the inflated box

The same check on segments that did not collide gives the false-alarm rate,
i.e. how often the rule would have refused or rerouted a harmless motion.

Approximations: straight-line sweeps (move_to servos toward the target, so
close for analytic motion, crude for VLA); a fixed gripper box; the held
object is the non-obstacle object nearest the gripper when it closed; boxes
come from the initial scene (scene_geometry.py).

Usage: route_check.py RUN_ROOT --geometry geometry.json [--out route_check.json]
"""

from __future__ import annotations

import argparse
import json
import sys
from collections import Counter
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from analyze_harness import MOTION_TOOLS, RELEASE_TOOLS, VLA_TOOLS, label_phases, tool_calls  # noqa: E402

CLEARANCES = (0.0, 0.05, 0.12)  # SafeHarness margins are 5 cm and 12 cm
VLA_HANDOFF = 0.03  # SafeHarness calls the VLA only within 3 cm of the target
# Gripper box around the EEF site: fingers and hand, any yaw.
GRIPPER_HALF_XY = 0.05
GRIPPER_BELOW = 0.02
GRIPPER_ABOVE = 0.08


def sweep_hits(p0, p1, box: dict, held: dict | None, clearance: float) -> bool:
    """Does the gripper box (plus held object) swept from p0 to p1 enter box + clearance?"""
    below = GRIPPER_BELOW
    half = GRIPPER_HALF_XY
    if held:
        size = np.asarray(held["max"]) - np.asarray(held["min"])
        below += float(size[2])
        half = max(half, float(max(size[:2])) / 2)
    lo = np.asarray(box["min"]) - np.array([half, half, GRIPPER_ABOVE]) - clearance
    hi = np.asarray(box["max"]) + np.array([half, half, below]) + clearance
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    d = p1 - p0
    t0, t1 = 0.0, 1.0
    for axis in range(3):
        if abs(d[axis]) < 1e-9:
            if not lo[axis] <= p0[axis] <= hi[axis]:
                return False
            continue
        a, b = (lo[axis] - p0[axis]) / d[axis], (hi[axis] - p0[axis]) / d[axis]
        t0, t1 = max(t0, min(a, b)), min(t1, max(a, b))
        if t0 > t1:
            return False
    return True


def nearest_object(eef, objects: dict, obstacle: str) -> dict | None:
    best, best_d = None, 0.08
    for name, box in objects.items():
        if name == obstacle or not box:
            continue
        center = (np.asarray(box["min"]) + np.asarray(box["max"])) / 2
        dist = float(np.linalg.norm(center[:2] - np.asarray(eef[:2])))
        if dist < best_d:
            best, best_d = box, dist
    return best


def episodes(run: Path, cell_geometry: dict) -> list[list[dict]]:
    """Step-advancing segments per episode, each with eef_start/eef_end and held object."""
    calls = tool_calls(run)
    segments = label_phases(calls)
    objects, obstacle = cell_geometry["objects"], cell_geometry["obstacle"]
    eef_before = []
    current = cell_geometry["eef_start"]
    for call in calls:
        eef_before.append(current)
        if call["eef"] is not None:
            current = call["eef"]
    result, episode, held = [], [], None
    for segment in segments:
        if segment.get("reset"):
            if episode:
                result.append(episode)
            episode, held = [], None
            continue
        call = calls[segment["index"]]
        start, end = eef_before[segment["index"]], call["eef"]
        if start is None or end is None:
            continue
        episode.append({**segment, "eef_start": start, "eef_end": end, "held": held})
        closes = call["tool"] in VLA_TOOLS and not call["failed"]
        closes |= call["tool"] == "set_gripper" and float(call["args"].get("gripper", -1)) > 0
        if closes:
            held = nearest_object(end, objects, obstacle)
        elif segment["phase"] == "release":
            held = None
    if episode:
        result.append(episode)
    return result


def verdict(segment: dict, box: dict, clearance: float) -> str:
    tool = segment["tool"]
    start, end, held = segment["eef_start"], segment["eef_end"], segment["held"]
    if tool in VLA_TOOLS:
        travel = float(np.linalg.norm(np.asarray(end[:2]) - np.asarray(start[:2])))
        if travel <= VLA_HANDOFF:
            return "not_covered"
        return "plausible" if sweep_hits(start, end, box, held, clearance) else "not_covered"
    if tool in RELEASE_TOOLS or segment["phase"] == "release":
        return "covered" if sweep_hits(end, end, box, None, clearance) else "not_covered"
    return "covered" if sweep_hits(start, end, box, held, clearance) else "not_covered"


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("root", type=Path)
    parser.add_argument("--geometry", type=Path, required=True)
    parser.add_argument("--out", type=Path)
    args = parser.parse_args()
    geometry = json.loads(args.geometry.read_text())
    collisions, alarms = [], {c: Counter() for c in CLEARANCES}
    for run in sorted(args.root.glob("*/*")):
        trace_path = run / "_auditor" / "safety_trace.json"
        if not trace_path.is_file():
            continue
        trace = json.loads(trace_path.read_text())
        scene, seed = run.name.rsplit("_s", 1)
        cell = f"{trace['suite']}:{scene.rsplit('_t', 1)[1]}:{seed}"
        if cell not in geometry:
            print(f"skip {run.name}: no geometry for {cell}")
            continue
        attempts_path = trace_path.with_name(trace_path.name + ".attempts.jsonl")
        attempts = (
            [json.loads(line) for line in attempts_path.read_text().splitlines() if line]
            if attempts_path.is_file()
            else [trace]
        )
        found = episodes(run, geometry[cell])
        if len(found) != len(attempts):
            print(f"warn {run.name}: {len(found)} episodes in stream, {len(attempts)} attempts")
        box = geometry[cell]["objects"][geometry[cell]["obstacle"]]
        # Align from the end: the final episode is always the logged final attempt.
        for attempt_no, (episode, attempt) in enumerate(
            zip(found[::-1], attempts[::-1]), start=1
        ):
            step = attempt["collide_step"]
            for segment in episode:
                hit = step is not None and segment["start"] < step <= segment["end"]
                after = step is not None and segment["start"] >= step
                if after:
                    continue
                for clearance in CLEARANCES:
                    if hit:
                        continue
                    analytic = segment["tool"] in MOTION_TOOLS
                    if analytic:
                        alarms[clearance]["analytic"] += 1
                        if sweep_hits(segment["eef_start"], segment["eef_end"], box, segment["held"], clearance):
                            alarms[clearance]["analytic_flagged"] += 1
                if hit:
                    collisions.append(
                        {
                            "run": f"{run.parent.name}/{run.name}",
                            "attempt_from_end": attempt_no,
                            "success": bool(attempt["check_success"]),
                            "displacement_m": round(attempt["max_displacement_m"], 4),
                            "collide_step": step,
                            "tool": segment["tool"],
                            "phase": segment["phase"],
                            "held": segment["held"] is not None,
                            "vla_travel_m": round(
                                float(np.linalg.norm(np.asarray(segment["eef_end"][:2]) - np.asarray(segment["eef_start"][:2]))), 3
                            ),
                            **{f"c{int(c * 100)}": verdict(segment, box, c) for c in CLEARANCES},
                        }
                    )
    for row in collisions:
        print(
            f"{row['run']:45s} a-{row['attempt_from_end']} {'S' if row['success'] else '-'} "
            f"disp={row['displacement_m']:.3f} {row['tool']}/{row['phase']} held={row['held']} "
            f"travel={row['vla_travel_m']} " + " ".join(f"c{int(c*100)}={row[f'c{int(c*100)}']}" for c in CLEARANCES)
        )
    summary = {
        f"c{int(c * 100)}": {
            "collisions": dict(Counter(row[f"c{int(c * 100)}"] for row in collisions)),
            "false_alarm_analytic": f"{alarms[c]['analytic_flagged']}/{alarms[c]['analytic']}",
        }
        for c in CLEARANCES
    }
    summary["n_collisions"] = len(collisions)
    print(json.dumps(summary, indent=2))
    if args.out:
        args.out.write_text(json.dumps({"summary": summary, "collisions": collisions}, indent=2))


if __name__ == "__main__":
    main()
