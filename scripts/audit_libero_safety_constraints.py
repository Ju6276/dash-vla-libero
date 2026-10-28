"""Read each LIBERO-Safety constraint on its own.

The official benchmark, its thresholds, and its BDDL files are not modified.
This script calls the existing predicate function once per constraint and
records that value before the official cost dictionary keeps only the last
predicate of each type.
"""

from __future__ import annotations

import json
import os
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
SAFETY_ROOT = ROOT / "third_party" / "LIBERO-Safety"
BENCHMARK = SAFETY_ROOT / "libero" / "libero"
CONFIG = ROOT / "runs" / "g5_libero" / "libero_config_safety"
OUT = ROOT / "runs" / "g5_libero" / "constraint_audit_stove.json"

os.environ.setdefault("MUJOCO_GL", "egl")
os.environ.setdefault("PYOPENGL_PLATFORM", "egl")
os.environ.pop("MUJOCO_EGL_DEVICE_ID", None)
sys.path.insert(0, str(SAFETY_ROOT))
sys.path.insert(0, str(SAFETY_ROOT / "third_party" / "robosuite-1.4"))


def ensure_config() -> None:
    CONFIG.mkdir(parents=True, exist_ok=True)
    datasets = ROOT / "runs" / "g5_libero" / "datasets_placeholder"
    datasets.mkdir(parents=True, exist_ok=True)
    text = "".join(
        f"{key}: {value}\n"
        for key, value in {
            "benchmark_root": BENCHMARK,
            "bddl_files": BENCHMARK / "bddl_files",
            "init_states": BENCHMARK / "init_files",
            "datasets": datasets,
            "assets": BENCHMARK / "assets",
        }.items()
    )
    (CONFIG / "config.yaml").write_text(text)
    os.environ["LIBERO_CONFIG_PATH"] = str(CONFIG)


def each_constraint(domain) -> list[dict]:
    rows = []
    for index, constraint in enumerate(domain.parsed_problem["constraints"]):
        value = int(bool(domain._eval_predicate(constraint)))
        rows.append(
            {
                "index": index,
                "predicate": str(constraint[0]),
                "arguments": [str(item) for item in constraint[1:]],
                "value": value,
            }
        )
    return rows


def collapsed(rows: list[dict]) -> dict:
    cost = {}
    for row in rows:
        cost[row["predicate"]] = row["value"]
    return cost


def snapshot(domain) -> dict:
    rows = each_constraint(domain)
    return {
        "per_constraint": rows,
        "collapsed_by_predicate_name": collapsed(rows),
        "official_cost_while_not_done": domain._check_constraint(False),
        "official_cost_if_success_step": domain._check_constraint(True),
    }


def qpos_span(sim, joint: str) -> tuple[int, int]:
    address = sim.model.get_joint_qpos_addr(joint)
    if isinstance(address, slice):
        return int(address.start), int(address.stop)
    if isinstance(address, tuple):
        return int(address[0]), int(address[1])
    start = int(address)
    return start, start + 1


def place(sim, obj, position: np.ndarray) -> None:
    start, end = qpos_span(sim, obj.joints[0])
    current = np.array(sim.data.qpos[start:end], dtype=np.float64)
    current[:3] = np.asarray(position, dtype=np.float64)
    sim.data.qpos[start:end] = current
    velocity = sim.model.get_joint_qvel_addr(obj.joints[0])
    if isinstance(velocity, slice):
        sim.data.qvel[velocity] = 0
    elif isinstance(velocity, tuple):
        sim.data.qvel[int(velocity[0]) : int(velocity[1])] = 0
    sim.forward()


def body_pos(domain, name: str) -> np.ndarray:
    return np.asarray(domain.sim.data.body_xpos[domain.obj_body_id[name]], dtype=np.float64)


def save_state(sim) -> tuple[np.ndarray, np.ndarray]:
    return sim.data.qpos.copy(), sim.data.qvel.copy()


def restore_state(sim, state: tuple[np.ndarray, np.ndarray]) -> None:
    sim.data.qpos[:] = state[0]
    sim.data.qvel[:] = state[1]
    sim.forward()


def robot_geom_report(domain) -> dict:
    sim = domain.sim
    indexes = []
    names = []
    for geom_index in range(sim.model.ngeom):
        name = sim.model.geom_id2name(geom_index)
        if not name or int(sim.model.geom_group[geom_index]) != 0:
            continue
        if "gripper" in name or "robot" in name:
            indexes.append(int(geom_index))
            names.append(name)
    sample_name = names[0] if names else None
    return {
        "count": len(indexes),
        "stored_type": type(indexes[0]).__name__ if indexes else None,
        "sample_name": sample_name,
        "sample_name_matches_stored_indexes": bool(sample_name in indexes) if sample_name else False,
        "names": names,
    }


def objects_touch(domain, first: str, second: str) -> bool:
    return bool(domain.check_contact(domain.get_object(first), domain.get_object(second)))


def search_object_contact(domain, mover: str, anchor: np.ndarray, target: str) -> dict:
    obj = domain.get_object(mover)
    offsets = np.linspace(-0.05, 0.05, 5)
    for dx in offsets:
        for dy in offsets:
            for dz in offsets:
                place(domain.sim, obj, anchor + np.array([dx, dy, dz], dtype=np.float64))
                if objects_touch(domain, mover, target):
                    return {
                        "found": True,
                        "offset": [float(dx), float(dy), float(dz)],
                        "mover_pos": [float(x) for x in body_pos(domain, mover)],
                    }
    return {"found": False, "offset": None, "mover_pos": [float(x) for x in body_pos(domain, mover)]}


def search_robot_contact(domain, mover: str, geom_names: list[str]) -> dict:
    obj = domain.get_object(mover)
    offsets = np.linspace(-0.03, 0.03, 5)
    for geom_name in geom_names[:12]:
        anchor = np.asarray(domain.sim.data.get_geom_xpos(geom_name), dtype=np.float64)
        for dx in offsets:
            for dy in offsets:
                for dz in offsets:
                    place(domain.sim, obj, anchor + np.array([dx, dy, dz], dtype=np.float64))
                    if domain.check_contact(geom_names, obj):
                        return {
                            "found": True,
                            "geom": geom_name,
                            "offset": [float(dx), float(dy), float(dz)],
                            "name_based_robot_contact": True,
                        }
    return {"found": False, "geom": None, "offset": None, "name_based_robot_contact": False}


def load_episode(level: int):
    import torch
    from libero.libero.benchmark import get_benchmark
    from libero.libero.envs import OffScreenRenderEnv

    suite = get_benchmark("obstacle_avoidance")()
    task = suite.get_task_by_level_id(level, 0)
    bddl = Path(suite.get_task_bddl_file_path_by_level_id(level, 0))
    init_path = (
        BENCHMARK
        / "init_files"
        / bddl.parent.relative_to(BENCHMARK / "bddl_files")
        / f"{bddl.stem}.pruned_init"
    )
    env = OffScreenRenderEnv(bddl_file_name=str(bddl), camera_heights=128, camera_widths=128)
    env.reset()
    loaded = torch.load(init_path, weights_only=False)
    if hasattr(loaded, "detach"):
        loaded = loaded.detach().cpu().numpy()
    loaded = np.asarray(loaded)
    init_state = loaded[0] if loaded.ndim > 1 else loaded
    env.set_init_state(np.asarray(init_state, dtype=np.float64))
    wait = np.zeros(7, dtype=np.float32)
    wait[-1] = -1.0
    for _ in range(15):
        env.step(wait)
    domain = env.env
    return env, domain, {
        "level": level,
        "task_name": task.name,
        "bddl_path": str(bddl),
        "init_state_path": str(init_path),
        "parsed_constraints": [
            [str(item) for item in constraint]
            for constraint in domain.parsed_problem["constraints"]
        ],
    }


def step_cost(env) -> dict:
    wait = np.zeros(7, dtype=np.float32)
    wait[-1] = -1.0
    result = env.step(wait)
    info = result[3] if len(result) == 4 else result[4]
    done = bool(result[2]) if len(result) == 4 else bool(result[2] or result[3])
    cost = info.get("cost") if isinstance(info, dict) else None
    return {"done": done, "info_cost": cost}


def audit_level(level: int) -> dict:
    env, domain, identity = load_episode(level)
    try:
        report = {
            **identity,
            "robot_geom_group_used_by_check_robot_contact": robot_geom_report(domain),
            "baseline_after_settle": snapshot(domain),
        }
        state = save_state(domain.sim)
        trials = []
        constraints = domain.parsed_problem["constraints"]
        contact_rows = [row for row in constraints if row[0] == "checkcontact" and len(row) == 3]
        robot_rows = [row for row in constraints if row[0] == "checkrobotcontact" and len(row) == 2]
        if len(contact_rows) >= 2:
            first = contact_rows[0]
            last = contact_rows[-1]
            mover, first_target = str(first[1]), str(first[2])
            last_target = str(last[2])
            isolated = body_pos(domain, first_target) + np.array([0.35, 0.0, 0.25])
            place(domain.sim, domain.get_object(first_target), isolated)
            first_hit = search_object_contact(domain, mover, isolated, first_target)
            first_hit["also_touches_last_target"] = objects_touch(domain, mover, last_target)
            first_hit["reading_before_step"] = snapshot(domain)
            first_hit["official_step"] = step_cost(env)
            first_hit["reading_after_official_step"] = snapshot(domain)
            first_hit["which"] = "first_checkcontact_only"
            trials.append(first_hit)
            restore_state(domain.sim, state)

            place(domain.sim, domain.get_object(first_target), isolated)
            last_anchor = body_pos(domain, last_target)
            last_hit = search_object_contact(domain, mover, last_anchor, last_target)
            last_hit["also_touches_first_target"] = objects_touch(domain, mover, first_target)
            last_hit["reading_before_step"] = snapshot(domain)
            last_hit["official_step"] = step_cost(env)
            last_hit["reading_after_official_step"] = snapshot(domain)
            last_hit["which"] = "last_checkcontact_only"
            trials.append(last_hit)
            restore_state(domain.sim, state)

        if robot_rows:
            robot_names = robot_geom_report(domain)["names"]
            target_name = str(robot_rows[0][1])
            robot_hit = search_robot_contact(domain, target_name, robot_names)
            robot_hit["which"] = "first_checkrobotcontact_object_on_robot_geom"
            robot_hit["reading"] = snapshot(domain)
            robot_hit["official_step"] = step_cost(env)
            robot_hit["reading_after_official_step"] = snapshot(domain)
            trials.append(robot_hit)
            restore_state(domain.sim, state)
        report["positive_controls"] = trials
        return report
    finally:
        env.close()


def main() -> None:
    ensure_config()
    import torch

    torch.cuda.init()
    payload = {
        "benchmark_modified": False,
        "levels": [audit_level(0), audit_level(1)],
    }
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps(payload, indent=2, sort_keys=True))
    print(json.dumps({"wrote": str(OUT)}, sort_keys=True))


if __name__ == "__main__":
    main()
