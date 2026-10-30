#!/usr/bin/env python3
"""Offline estimate: which original-Harness collisions a SafeHarness-style check covers.

SafeHarness (arXiv 2609.20822) grounds objects as boxes, verifies every
analytic waypoint route against the obstacle box (a segment collides if the
gripper or held object passes within 12 cm of the obstacle or less than 5 cm
above it), and invokes the frozen VLA only once the gripper is within 3 cm of
the target. We have no code for it; this replays the same check the online B2
arm uses (robots/libero/route_layer.py) on the recorded trajectories, with
simulator boxes instead of SAM 3 boxes:

  analytic motion   covered if the straight start->end segment fails the check
  VLA call          not covered if the VLA started within 3 cm (horizontal) of
                    where it ended, i.e. SafeHarness would also have run it
                    there; otherwise "plausible": SafeHarness would have done
                    that travel analytically
  release           covered if the release pose itself fails the check

The same check on segments that did not collide gives the share of harmless
B0 motions SafeHarness would have refused or rerouted.

Approximations: straight-line segments (move_to servos toward the target, so
close for analytic motion, crude for VLA); the held object is the
non-obstacle object nearest the gripper when it closed; every non-obstacle
object counts as a possible contact target; boxes come from the initial scene
(scene_geometry.py).

Usage: route_check.py RUN_ROOT --geometry geometry.json [--out route_check.json]
"""

from __future__ import annotations

import argparse
import json
import sys
from collections import Counter
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parent))
from analyze_harness import MOTION_TOOLS, RELEASE_TOOLS, VLA_TOOLS, label_phases, tool_calls  # noqa: E402
from robots.libero.route_layer import VLA_HANDOFF, SafeHarnessLayer  # noqa: E402


class _NoPrimitives:
    _step_monitor = None


def wrist_yaw(quat) -> float:
    """rotate_wrist yaw (EEF x-axis) from an xyzw quaternion; the reset pose is ~0."""
    if quat is None:
        return 0.0
    matrix = Rotation.from_quat(quat).as_matrix()
    return float(np.arctan2(matrix[1, 0], matrix[0, 0]))


def scene_layer(cell_geometry: dict) -> SafeHarnessLayer:
    """The B2 check with simulator boxes: the obstacle, and every other object as a target."""
    layer = SafeHarnessLayer(_NoPrimitives())
    for name, box in cell_geometry["objects"].items():
        if box:
            role = "obstacle" if name == cell_geometry["obstacle"] else "target"
            layer.boxes[name] = {"label": name, "role": role, **box}
    return layer


def fails(layer: SafeHarnessLayer, start, end, held: str | None, yaws) -> bool:
    layer.held = held
    return not layer.check_segment(start, end, yaws)["ok"]


def nearest_object(eef, objects: dict, obstacle: str) -> str | None:
    best, best_d = None, 0.08
    for name, box in objects.items():
        if name == obstacle or not box:
            continue
        center = (np.asarray(box["min"]) + np.asarray(box["max"])) / 2
        dist = float(np.linalg.norm(center[:2] - np.asarray(eef[:2])))
        if dist < best_d:
            best, best_d = name, dist
    return best


def episodes(run: Path, cell_geometry: dict) -> list[list[dict]]:
    """Step-advancing segments per episode, each with eef_start/eef_end and held object."""
    calls = tool_calls(run)
    segments = label_phases(calls)
    objects, obstacle = cell_geometry["objects"], cell_geometry["obstacle"]
    eef_before, yaw_before = [], []
    current, yaw = cell_geometry["eef_start"], 0.0
    for call in calls:
        eef_before.append(current)
        yaw_before.append(yaw)
        if call["eef"] is not None:
            current = call["eef"]
            yaw = wrist_yaw(call.get("eef_quat"))
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
        yaws = (yaw_before[segment["index"]], wrist_yaw(call.get("eef_quat")))
        episode.append({**segment, "eef_start": start, "eef_end": end, "held": held, "yaws": yaws})
        closes = call["tool"] in VLA_TOOLS and not call["failed"]
        closes |= call["tool"] == "set_gripper" and float(call["args"].get("gripper", -1)) > 0
        if closes:
            held = nearest_object(end, objects, obstacle)
        elif segment["phase"] == "release":
            held = None
    if episode:
        result.append(episode)
    return result


def verdict(segment: dict, layer: SafeHarnessLayer) -> str:
    tool = segment["tool"]
    start, end, held = segment["eef_start"], segment["eef_end"], segment["held"]
    if tool in VLA_TOOLS:
        travel = float(np.linalg.norm(np.asarray(end[:2]) - np.asarray(start[:2])))
        return "not_covered" if travel <= VLA_HANDOFF else "plausible"
    if tool in RELEASE_TOOLS or segment["phase"] == "release":
        return "covered" if fails(layer, end, end, None, segment["yaws"][1:]) else "not_covered"
    return "covered" if fails(layer, start, end, held, segment["yaws"]) else "not_covered"


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("root", type=Path)
    parser.add_argument("--audit", type=Path, help="AUDIT_ROOT/<mode>/<run>/safety_trace.json")
    parser.add_argument("--geometry", type=Path, required=True)
    parser.add_argument("--out", type=Path)
    args = parser.parse_args()
    geometry = json.loads(args.geometry.read_text())
    collisions, alarms = [], Counter()
    for run in sorted(args.root.glob("*/*")):
        trace_path = run / "_auditor" / "safety_trace.json"
        if not trace_path.is_file() and args.audit is not None:
            trace_path = args.audit / run.parent.name / run.name / "safety_trace.json"
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
        layer = scene_layer(geometry[cell])
        # Align from the end: the final episode is always the logged final attempt.
        for attempt_no, (episode, attempt) in enumerate(
            zip(found[::-1], attempts[::-1]), start=1
        ):
            step = attempt["collide_step"]
            for segment in episode:
                if step is not None and segment["start"] >= step:
                    continue
                if step is None or not segment["start"] < step <= segment["end"]:
                    if segment["tool"] in MOTION_TOOLS:
                        alarms["analytic"] += 1
                        alarms["analytic_flagged"] += fails(
                            layer, segment["eef_start"], segment["eef_end"], segment["held"], segment["yaws"]
                        )
                    continue
                travel = np.asarray(segment["eef_end"][:2]) - np.asarray(segment["eef_start"][:2])
                collisions.append(
                    {
                        "run": f"{run.parent.name}/{run.name}",
                        "attempt_from_end": attempt_no,
                        "success": bool(attempt["check_success"]),
                        "displacement_m": round(attempt["max_displacement_m"], 4),
                        "collide_step": step,
                        "tool": segment["tool"],
                        "phase": segment["phase"],
                        "held": segment["held"],
                        "travel_xy_m": round(float(np.linalg.norm(travel)), 3),
                        "verdict": verdict(segment, layer),
                    }
                )
    for row in collisions:
        print(
            f"{row['run']:45s} a-{row['attempt_from_end']} {'S' if row['success'] else '-'} "
            f"disp={row['displacement_m']:.3f} {row['tool']}/{row['phase']} held={row['held']} "
            f"travel={row['travel_xy_m']} {row['verdict']}"
        )
    summary = {
        "n_collisions": len(collisions),
        "collisions": dict(Counter(row["verdict"] for row in collisions)),
        "harmless_analytic_refused": f"{alarms['analytic_flagged']}/{alarms['analytic']}",
    }
    print(json.dumps(summary, indent=2))
    if args.out:
        args.out.write_text(json.dumps({"summary": summary, "collisions": collisions}, indent=2))


if __name__ == "__main__":
    main()
