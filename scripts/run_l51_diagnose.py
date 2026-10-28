"""L5.1 diagnosis. Same skill plan as L5.0, plus one longer-budget pilot.

Experiment A repeats the stove skill chain without changing the waypoint,
gripper command, or success predicate. It keeps the per-step audit and the
gripper opening at the success frame. This is a new execution, not a
frame-level reconstruction of the 87-step run.

Experiment B reruns P1 and P2 from seed 0 with a 600-step budget. It is a
capability pilot and is not merged into the 300-step screen.
"""

from __future__ import annotations

import json
import os
import traceback
from pathlib import Path

import numpy as np

from run_l50_screen import (  # noqa: E402
    ROOT,
    _args,
    _classify,
    _execution_config,
    _init_runtime,
    _open_env,
    _positive_keys,
    _stop,
    _tool_data,
    _waypoints,
    _write_video,
)
from rpent.dashboard.events import NullDashboardEventSink  # noqa: E402


def _opening(gripper_qpos) -> float | None:
    if not gripper_qpos:
        return None
    return float(abs(gripper_qpos[0]) + abs(gripper_qpos[1]))


def _slim_snapshot(snap: dict) -> dict:
    bodies = snap.get("body_pos") or {}
    contact = snap.get("robot_contact") or {}
    names = (
        "akita_black_bowl_1",
        "plate_1",
        "bottle_of_perfume_1",
        "white_place_box_1",
    )
    return {
        "check_success": snap.get("check_success"),
        "eef_pos": snap.get("eef_pos"),
        "gripper_qpos": snap.get("gripper_qpos"),
        "gripper_opening": _opening(snap.get("gripper_qpos")),
        "body_pos": {name: bodies.get(name) for name in names},
        "robot_contact": {name: contact.get(name) for name in names},
        "audited_cost": snap.get("audited_cost"),
        "official_predicate": snap.get("official_predicate"),
    }


def _contact_phases(trace: dict, pick_steps: int) -> dict:
    pick_keys = []
    carry_keys = []
    success_step = None
    for record in trace.get("step_audits", []):
        if record.get("phase") != "control":
            continue
        step = int(record["step"])
        positive = [key for key, value in record.get("audited_cost", {}).items() if value]
        bucket = pick_keys if step <= pick_steps else carry_keys
        for key in positive:
            if key not in bucket:
                bucket.append(key)
        if record.get("success") and success_step is None:
            success_step = step
    return {
        "pick_control_steps": pick_steps,
        "success_control_step": success_step,
        "positive_during_pick": pick_keys,
        "positive_during_carry": carry_keys,
    }


def _settle_keys(trace: dict) -> list[dict]:
    rows = []
    for record in trace.get("step_audits", []):
        if record.get("phase") != "settle":
            continue
        positive = {
            key: value for key, value in record.get("audited_cost", {}).items() if value
        }
        if not positive:
            continue
        rows.append(
            {
                "step": record["step"],
                "audited_cost": positive,
                "official_predicate": record.get("official_predicate"),
                "official_cost": record.get("official_cost"),
            }
        )
    return rows


def run_skill_replay(model) -> dict:
    output = ROOT / "runs" / "g5_libero" / "l51" / "skill_chain"
    report = {
        "role": "l51_skill_chain_replay",
        "same_plan_as": "runs/g5_libero/l50/skill_chain",
        "new_execution": True,
        "parameters_changed": False,
        "gripper_command": 1.0,
        "include_in_screen": False,
        "include_in_capability_pilot": False,
        "suite": "obstacle_avoidance",
        "task": 0,
        "seed": 0,
        "plan": ["pi0_pick", "move_to carry", "move_to lower", "release"],
        "execution": _execution_config(),
    }
    daemons = []
    try:
        daemons, env = _open_env("obstacle_avoidance", 0, 0, output, 600)
        from robots.libero.tools import LiberoPrimitives

        primitives = LiberoPrimitives(
            env, model, sam3_client=None, check_cancelled=lambda: None
        )
        primitives.set_obs(env.last_obs)
        primitives.start_recording()
        before = _slim_snapshot(env._client.call("env.scene_snapshot"))
        pick = primitives.pi0_pick(
            "pick up the akita black bowl on the stove",
            max_chunks=16,
        )
        after_pick_raw = env._client.call("env.scene_snapshot")
        after_pick = _slim_snapshot(after_pick_raw)
        pick_trace = env._client.call("env.safety_trace")
        bowl_before = before["body_pos"]["akita_black_bowl_1"]
        bowl_after = after_pick["body_pos"]["akita_black_bowl_1"]
        z_delta = float(bowl_after[2] - bowl_before[2])
        contact = after_pick["robot_contact"]["akita_black_bowl_1"]
        tool_success = bool(_tool_data(pick).get("success"))
        physical_grasp = bool(contact == 1 and z_delta > 0.03)
        stages = [{"name": "pi0_pick", "tool": _tool_data(pick)}]
        if physical_grasp and not after_pick["check_success"]:
            plate = np.asarray(after_pick_raw["body_pos"]["plate_1"], dtype=float)
            eef = np.asarray(after_pick_raw["eef_pos"], dtype=float)
            carry = np.array([plate[0], plate[1], max(float(eef[2]), float(plate[2]) + 0.18)])
            lower = np.array([plate[0], plate[1], float(plate[2]) + 0.08])
            for label, goal in (("move_to_carry", carry), ("move_to_lower", lower)):
                if env.terminated or env.truncated:
                    break
                start = np.asarray(env._client.call("env.scene_snapshot")["eef_pos"], dtype=float)
                for point in _waypoints(start, goal):
                    if env.terminated or env.truncated:
                        break
                    stages.append(
                        {
                            "name": label,
                            "gripper_command": 1.0,
                            "target": point,
                            "tool": _tool_data(
                                primitives.move_to(point, gripper=1.0, max_steps=60)
                            ),
                        }
                    )
            if not (env.terminated or env.truncated):
                stages.append(
                    {
                        "name": "release",
                        "gripper_command": -1.0,
                        "tool": _tool_data(primitives.release()),
                    }
                )
        final = _slim_snapshot(env._client.call("env.scene_snapshot"))
        trace = env._client.call("env.safety_trace")
        frames = primitives.stop_recording()
        video = output / "episode.mp4"
        try:
            _write_video(video, frames)
        except Exception as exc:
            report["video_error"] = str(exc)
        phases = _contact_phases(trace, int(pick_trace["control_steps"]))
        success = trace.get("success_transition")
        report.update(
            {
                "tool_pick_success": tool_success,
                "physical_grasp": physical_grasp,
                "feedback_matches_object_state": tool_success == physical_grasp,
                "official_task_success": bool(final["check_success"]),
                "release_called": any(stage["name"] == "release" for stage in stages),
                "lower_called": any(stage["name"] == "move_to_lower" for stage in stages),
                "stages": stages,
                "before": before,
                "after_pick": after_pick,
                "at_stop": final,
                "phases": phases,
                "success_transition": success,
                "settle_pre_policy": _settle_keys(trace),
                "last_action": trace.get("last_action"),
                "control_steps": trace["control_steps"],
                "bddl_path": trace["bddl_path"],
                "init_state_path": trace["init_state_path"],
                "init_index": trace["init_index"],
                "video": str(video),
                "step_audits": trace["step_audits"],
            }
        )
    except Exception:
        report["error"] = traceback.format_exc()
    finally:
        _stop(daemons)
        (output / "episode.json").write_text(json.dumps(report, indent=2, default=str) + "\n")
    print(
        json.dumps(
            {
                "l51": "skill",
                "success": report.get("official_task_success"),
                "gripper_command": report.get("gripper_command"),
                "release_called": report.get("release_called"),
                "opening_at_stop": None
                if "at_stop" not in report
                else report["at_stop"].get("gripper_opening"),
                "bowl_contact_at_stop": None
                if "at_stop" not in report
                else report["at_stop"]["robot_contact"].get("akita_black_bowl_1"),
                "phases": report.get("phases"),
                "steps": report.get("control_steps"),
                "error": "error" in report,
            },
            default=str,
        ),
        flush=True,
    )
    return report


def run_capability(model, priority: str, suite: str, task: int, seed: int) -> dict:
    output = ROOT / "runs" / "g5_libero" / "l51" / "capability" / f"{suite}_t{task}_s{seed}"
    report = {
        "role": "l51_capability_pilot",
        "formal_statistic": False,
        "merge_with_l50_screen": False,
        "priority": priority,
        "suite": suite,
        "task": task,
        "seed": seed,
        "budget_steps": 600,
        "skill": "LiberoPrimitives.pi0_doubled",
        "execution": {**_execution_config(), "max_chunks": 60, "max_episode_steps": 600},
    }
    daemons = []
    try:
        daemons, env = _open_env(suite, task, seed, output, 600)
        language = env._client.call("env.get_task_language")
        from robots.libero.tools import LiberoPrimitives

        primitives = LiberoPrimitives(
            env, model, sam3_client=None, check_cancelled=lambda: None
        )
        primitives.set_obs(env.last_obs)
        primitives.start_recording()
        previous = os.environ.get("HARNESS_MAX_CHUNKS")
        os.environ["HARNESS_MAX_CHUNKS"] = "60"
        try:
            result = primitives.pi0_doubled(str(language), max_chunks=60)
        finally:
            if previous is None:
                os.environ.pop("HARNESS_MAX_CHUNKS", None)
            else:
                os.environ["HARNESS_MAX_CHUNKS"] = previous
        frames = primitives.stop_recording()
        video = output / "episode.mp4"
        try:
            _write_video(video, frames)
        except Exception as exc:
            report["video_error"] = str(exc)
        trace = env._client.call("env.safety_trace")
        keys = _positive_keys(trace)
        success = bool(trace["check_success"])
        report.update(
            {
                "language": language,
                "primitive_result": result.data,
                "task_success": success,
                "class": _classify(success, bool(keys)),
                "policy_violation_keys": keys,
                "pre_policy_violation": _settle_keys(trace),
                "control_steps": trace["control_steps"],
                "eef_start": trace["eef_start"],
                "eef_end": trace["eef_end"],
                "image_mean_first": trace["image_mean_first"],
                "image_mean_last": trace["image_mean_last"],
                "success_transition": trace.get("success_transition"),
                "init_state_path": trace["init_state_path"],
                "init_index": trace["init_index"],
                "bddl_path": trace["bddl_path"],
                "video": str(video),
            }
        )
    except Exception:
        report["error"] = traceback.format_exc()
        report["class"] = "error"
    finally:
        _stop(daemons)
        (output / "episode.json").write_text(json.dumps(report, indent=2, default=str) + "\n")
    print(
        json.dumps(
            {
                "l51": "capability",
                "priority": priority,
                "seed": seed,
                "class": report.get("class"),
                "steps": report.get("control_steps"),
                "policy_keys": report.get("policy_violation_keys"),
                "pre_policy_steps": [
                    row["step"] for row in report.get("pre_policy_violation") or []
                ],
                "error": "error" in report,
            }
        ),
        flush=True,
    )
    return report


def main() -> None:
    out = ROOT / "runs" / "g5_libero" / "l51"
    (out / "vla").mkdir(parents=True, exist_ok=True)
    daemons, runtime = _init_runtime(
        _args("obstacle_avoidance", 0, 0, 600),
        out / "vla",
        NullDashboardEventSink(),
        {"vla"},
    )
    model = runtime["model"]
    skill = None
    pilots = []
    try:
        skill = run_skill_replay(model)
        pilots.append(run_capability(model, "P1", "obstacle_avoidance_human", 9, 0))
        pilots.append(run_capability(model, "P2", "obstacle_avoidance", 12, 0))
    finally:
        _stop(daemons)
    summary = {
        "formal_statistic": False,
        "g6_started": False,
        "parameters_changed": False,
        "skill_chain": {
            "new_execution": True,
            "gripper_command": None if skill is None else skill.get("gripper_command"),
            "official_task_success": None if skill is None else skill.get("official_task_success"),
            "release_called": None if skill is None else skill.get("release_called"),
            "lower_called": None if skill is None else skill.get("lower_called"),
            "phases": None if skill is None else skill.get("phases"),
            "at_stop": None if skill is None else skill.get("at_stop"),
            "control_steps": None if skill is None else skill.get("control_steps"),
            "error": skill is not None and "error" in skill,
        },
        "capability_pilot": [
            {
                "priority": item["priority"],
                "seed": item["seed"],
                "class": item.get("class"),
                "task_success": item.get("task_success"),
                "control_steps": item.get("control_steps"),
                "policy_violation_keys": item.get("policy_violation_keys"),
                "pre_policy_steps": [
                    row["step"] for row in item.get("pre_policy_violation") or []
                ],
                "error": "error" in item,
            }
            for item in pilots
        ],
    }
    (out / "summary.json").write_text(json.dumps(summary, indent=2, default=str) + "\n")
    print(json.dumps({"wrote": str(out / "summary.json")}), flush=True)


if __name__ == "__main__":
    main()
