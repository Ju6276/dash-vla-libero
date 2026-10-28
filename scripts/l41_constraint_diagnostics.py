"""Diagnostic contacts for the LIBERO-Safety constraint auditor.

This file does not change the benchmark, and its results are not baseline
episodes. It checks that the env-process auditor reads each constraint on its
own, including the step where success becomes true.
"""

from __future__ import annotations

import json
import os
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
HARNESS = ROOT / "vendor" / "RPent-harness"
SAFETY_ROOT = ROOT / "third_party" / "LIBERO-Safety"
OUT = ROOT / "runs" / "g5_libero" / "l41_diagnostic_controls.json"

os.environ.setdefault("MUJOCO_GL", "egl")
os.environ.setdefault("PYOPENGL_PLATFORM", "egl")
os.environ.pop("MUJOCO_EGL_DEVICE_ID", None)
os.environ.setdefault(
    "LIBERO_CONFIG_PATH",
    str(ROOT / "runs" / "g5_libero" / "libero_config_safety"),
)
sys.path.insert(0, str(HARNESS))
sys.path.insert(0, str(SAFETY_ROOT))
sys.path.insert(0, str(SAFETY_ROOT / "third_party" / "robosuite-1.4"))

from robots.libero.safety_env import SafetyRpcEnv, read_constraint_audit  # noqa: E402


def qpos_span(sim, joint: str) -> tuple[int, int]:
    address = sim.model.get_joint_qpos_addr(joint)
    if isinstance(address, slice):
        return int(address.start), int(address.stop)
    if isinstance(address, tuple):
        return int(address[0]), int(address[1])
    start = int(address)
    return start, start + 1


def place(sim, obj, position: np.ndarray) -> bool:
    joints = list(getattr(obj, "joints", []) or [])
    if not joints:
        return False
    start, end = qpos_span(sim, joints[0])
    if end - start < 7:
        return False
    current = np.array(sim.data.qpos[start:end], dtype=np.float64)
    current[:3] = np.asarray(position, dtype=np.float64)
    sim.data.qpos[start:end] = current
    velocity = sim.model.get_joint_qvel_addr(joints[0])
    if isinstance(velocity, slice):
        sim.data.qvel[velocity] = 0
    elif isinstance(velocity, tuple):
        sim.data.qvel[int(velocity[0]) : int(velocity[1])] = 0
    sim.forward()
    return True


def save_state(sim) -> tuple[np.ndarray, np.ndarray]:
    return np.array(sim.data.qpos), np.array(sim.data.qvel)


def restore_state(sim, state: tuple[np.ndarray, np.ndarray]) -> None:
    sim.data.qpos[:] = state[0]
    sim.data.qvel[:] = state[1]
    sim.forward()


def body_pos(domain, name: str) -> np.ndarray:
    return np.asarray(domain.sim.data.body_xpos[domain.obj_body_id[name]], dtype=np.float64)


def hold_pose(domain) -> None:
    controller = domain.robots[0].controller
    controller.update(force=True)
    controller.reset_goal()


def zero_action() -> np.ndarray:
    action = np.zeros(7, dtype=np.float32)
    action[-1] = -1.0
    return action


def signature(domain) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    sim = domain.sim
    ctrl = np.array(sim.data.ctrl) if sim.data.ctrl is not None else np.zeros(0)
    return np.array(sim.data.qpos), np.array(sim.data.qvel), ctrl


def max_abs(left: np.ndarray, right: np.ndarray) -> float:
    if left.shape != right.shape:
        return float("inf")
    if left.size == 0:
        return 0.0
    return float(np.max(np.abs(left - right)))


def open_env(level: int, *, audit: bool = True) -> SafetyRpcEnv:
    task = 0 if level == 0 else 5
    return SafetyRpcEnv("obstacle_avoidance", task, 0, 30, audit=audit)


def domain_of(env: SafetyRpcEnv):
    return env._sim.env


def control_step(env: SafetyRpcEnv) -> dict:
    env.terminated = False
    env.truncated = False
    before = len(env.step_audits)
    action = zero_action()
    sent = action.copy()
    env.step(action)
    if not np.array_equal(action, sent):
        raise RuntimeError("auditor changed the action array")
    added = env.step_audits[before:]
    control = [record for record in added if record["phase"] == "control"]
    if len(control) != 1:
        raise RuntimeError(f"expected one new control audit, got {len(control)}")
    return control[0]


def isolated(record: dict, target: str) -> bool:
    audited = record["audited_cost"]
    if audited.get(target) != 1:
        return False
    return all(value == 0 for key, value in audited.items() if key != target)


def summarize(record: dict, target: str) -> dict:
    return {
        "target": target,
        "isolated": isolated(record, target),
        "official_cost": record["official_cost"],
        "official_predicate": record["official_predicate"],
        "audited_cost": record["audited_cost"],
        "success": record["success"],
    }


def park_others(domain, keep: set[str]) -> None:
    origin = np.array([0.15, 0.85, 1.55], dtype=np.float64)
    index = 0
    for name, obj in domain.objects_dict.items():
        if name in keep:
            continue
        if place(domain.sim, obj, origin + np.array([0.12 * index, 0.0, 0.0])):
            index += 1


def prepare_pair(domain, mover: str, target: str) -> np.ndarray:
    park_others(domain, {mover, target})
    current = body_pos(domain, target)
    target_obj = domain.get_object(target)
    if place(domain.sim, target_obj, current + np.array([0.0, 0.48, 0.32])):
        return body_pos(domain, target)
    return current


def object_contact(env: SafetyRpcEnv, mover: str, target: str, key: str) -> dict:
    domain = domain_of(env)
    state = save_state(domain.sim)
    found = None
    for dx in np.linspace(-0.04, 0.04, 5):
        for dy in np.linspace(-0.04, 0.04, 5):
            for dz in np.linspace(-0.04, 0.04, 5):
                restore_state(domain.sim, state)
                anchor = prepare_pair(domain, mover, target)
                if not place(
                    domain.sim,
                    domain.get_object(mover),
                    anchor + np.array([dx, dy, dz]),
                ):
                    restore_state(domain.sim, state)
                    return {"target": key, "isolated": False, "error": f"{mover} has no free joint"}
                reading = read_constraint_audit(domain)
                if reading["audited_cost"].get(key) == 1 and all(
                    value == 0 for name, value in reading["audited_cost"].items() if name != key
                ):
                    found = np.array([dx, dy, dz], dtype=np.float64)
                    break
            if found is not None:
                break
        if found is not None:
            break
    if found is None:
        restore_state(domain.sim, state)
        return {"target": key, "isolated": False, "error": "no isolated contact before the step"}
    restore_state(domain.sim, state)
    anchor = prepare_pair(domain, mover, target)
    place(domain.sim, domain.get_object(mover), anchor + found)
    record = control_step(env)
    result = summarize(record, key)
    result["offset"] = [float(value) for value in found]
    restore_state(domain.sim, state)
    env.terminated = False
    env.truncated = False
    return result


def site_names(sim) -> list[str]:
    names = []
    for index in range(sim.model.nsite):
        name = sim.model.site_id2name(index)
        if name:
            names.append(name)
    return names


def arm_columns(sim) -> list[tuple[int, int]]:
    columns = []
    for index in range(sim.model.njnt):
        name = sim.model.joint_id2name(index)
        if not name or "robot0_joint" not in name:
            continue
        qpos = sim.model.get_joint_qpos_addr(name)
        qvel = sim.model.get_joint_qvel_addr(name)
        if isinstance(qpos, (int, np.integer)) and isinstance(qvel, (int, np.integer)):
            columns.append((int(qpos), int(qvel)))
    return columns


def site_jacobian(sim, site: str) -> np.ndarray:
    jac = np.array(sim.data.get_site_jacp(site), dtype=np.float64)
    if jac.ndim == 1:
        jac = jac.reshape(3, -1)
    return jac


def move_site(sim, site: str, target: np.ndarray) -> bool:
    columns = arm_columns(sim)
    if not columns:
        return False
    target = np.asarray(target, dtype=np.float64)
    for _ in range(60):
        sim.forward()
        error = target - np.array(sim.data.get_site_xpos(site), dtype=np.float64)
        if np.linalg.norm(error) < 0.01:
            sim.data.qvel[:] = 0
            sim.forward()
            return True
        jac = site_jacobian(sim, site)
        used = jac[:, [qvel for _, qvel in columns]]
        delta = used.T @ np.linalg.solve(used @ used.T + 1e-3 * np.eye(3), error)
        for (qpos_index, _), value in zip(columns, delta):
            sim.data.qpos[qpos_index] = float(sim.data.qpos[qpos_index] + value)
        sim.data.qvel[:] = 0
    sim.forward()
    error = target - np.array(sim.data.get_site_xpos(site), dtype=np.float64)
    return bool(np.linalg.norm(error) < 0.02)


def embed_robot_contact(env: SafetyRpcEnv, object_name: str, key: str) -> dict | None:
    domain = domain_of(env)
    obj = domain.get_object(object_name)
    joints = list(getattr(obj, "joints", []) or [])
    if not joints:
        return None
    start, end = qpos_span(domain.sim, joints[0])
    if end - start < 7:
        return None
    state = save_state(domain.sim)
    anchor = np.array(domain.sim.data.get_geom_xpos("robot0_link0_collision"), dtype=np.float64)
    found = None
    for dx in np.linspace(-0.02, 0.02, 3):
        for dy in np.linspace(-0.02, 0.02, 3):
            for dz in np.linspace(-0.02, 0.02, 3):
                restore_state(domain.sim, state)
                park_others(domain, {object_name})
                place(domain.sim, obj, anchor + np.array([dx, dy, dz]))
                reading = read_constraint_audit(domain)
                if reading["audited_cost"].get(key) != 1 or any(
                    value for name, value in reading["audited_cost"].items() if name != key
                ):
                    continue
                record = control_step(env)
                if isolated(record, key):
                    found = summarize(record, key)
                    found["offset"] = [float(dx), float(dy), float(dz)]
                    found["site"] = "robot0_link0_collision"
                    break
            if found:
                break
        if found:
            break
    restore_state(domain.sim, state)
    env.terminated = False
    env.truncated = False
    if found is None:
        return {
            "target": key,
            "isolated": False,
            "error": "static robot geom contact did not survive the control step",
        }
    return found


def robot_contact(env: SafetyRpcEnv, object_name: str, key: str) -> dict:
    embedded = embed_robot_contact(env, object_name, key)
    if embedded is not None and embedded.get("isolated"):
        return embedded
    domain = domain_of(env)
    state = save_state(domain.sim)
    sites = site_names(domain.sim)
    site = next(
        (name for name in ("gripper0_grip_site", "grip_site", "gripper0_ee_site") if name in sites),
        None,
    )
    if site is None:
        restore_state(domain.sim, state)
        return {"target": key, "isolated": False, "error": "grip site missing", "sites": sites[:20]}
    found = None
    for dx in np.linspace(-0.02, 0.02, 3):
        for dy in np.linspace(-0.02, 0.02, 3):
            for dz in np.linspace(-0.02, 0.02, 3):
                restore_state(domain.sim, state)
                park_others(domain, {object_name})
                current = body_pos(domain, object_name)
                if place(domain.sim, domain.get_object(object_name), current + np.array([0.0, 0.48, 0.32])):
                    current = body_pos(domain, object_name)
                if not move_site(domain.sim, site, current + np.array([dx, dy, dz])):
                    continue
                reading = read_constraint_audit(domain)
                if reading["audited_cost"].get(key) != 1 or any(
                    value for name, value in reading["audited_cost"].items() if name != key
                ):
                    continue
                hold_pose(domain)
                record = control_step(env)
                if isolated(record, key):
                    found = summarize(record, key)
                    found["offset"] = [float(dx), float(dy), float(dz)]
                    found["site"] = site
                    break
            if found:
                break
        if found:
            break
    restore_state(domain.sim, state)
    env.terminated = False
    env.truncated = False
    if found is None:
        return {
            "target": key,
            "isolated": False,
            "error": "gripper contact did not survive the control step",
            "site": site,
        }
    return found


def success_step(env: SafetyRpcEnv, mover: str, target: str, key: str) -> dict:
    domain = domain_of(env)
    state = save_state(domain.sim)
    anchor = body_pos(domain, target)
    held = None
    for dx in np.linspace(-0.04, 0.04, 5):
        for dy in np.linspace(-0.04, 0.04, 5):
            for dz in np.linspace(-0.04, 0.04, 5):
                restore_state(domain.sim, state)
                park_others(domain, {mover, target})
                if not place(domain.sim, domain.get_object(mover), anchor + np.array([dx, dy, dz])):
                    break
                reading = read_constraint_audit(domain)
                if reading["audited_cost"].get(key) != 1:
                    continue
                original = domain._check_success
                domain._check_success = lambda: True
                try:
                    record = control_step(env)
                finally:
                    domain._check_success = original
                env.terminated = False
                env.truncated = False
                if record["audited_cost"].get(key) == 1 and record["official_cost"] == {}:
                    held = summarize(record, key)
                    held["offset"] = [float(dx), float(dy), float(dz)]
                    held["official_cost_empty_on_success"] = True
                    break
            if held:
                break
        if held:
            break
    restore_state(domain.sim, state)
    if held is None:
        return {
            "target": key,
            "isolated": False,
            "error": "success step did not keep the audited contact",
        }
    return held


def negative(env: SafetyRpcEnv) -> dict:
    settle = [record for record in env.step_audits if record["phase"] == "settle"]
    control = control_step(env)
    audited_clear = all(
        value == 0
        for record in [*settle, control]
        for value in record["audited_cost"].values()
    )
    return {
        "settle_steps": len(settle),
        "audited_clear": audited_clear,
        "control": summarize(control, next(iter(control["audited_cost"]))),
        "official_cost": control["official_cost"],
    }


def read_only() -> dict:
    env = open_env(0, audit=True)
    domain = domain_of(env)
    before = signature(domain)
    image = np.array(env._raw["agentview_image"])
    state = np.array(env._raw["robot0_eef_pos"])
    read_constraint_audit(domain)
    after = signature(domain)
    image_after = np.array(env._raw["agentview_image"])
    state_after = np.array(env._raw["robot0_eef_pos"])
    during = {
        "qpos": max_abs(before[0], after[0]),
        "qvel": max_abs(before[1], after[1]),
        "ctrl": max_abs(before[2], after[2]),
        "image": max_abs(image, image_after),
        "eef": max_abs(state, state_after),
    }
    action = zero_action()
    env.step(action.copy())
    state = save_state(domain.sim)

    def replay(audit: bool) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        restore_state(domain.sim, state)
        hold_pose(domain)
        env.terminated = False
        env.truncated = False
        env.audit = audit
        env.step(action.copy())
        return (
            np.array(domain.sim.data.qpos),
            np.array(env._raw["agentview_image"]),
            np.array(env._raw["robot0_eef_pos"]),
        )

    on_qpos, on_image, on_eef = replay(True)
    off_qpos, off_image, off_eef = replay(False)
    paired = {
        "qpos": max_abs(on_qpos, off_qpos),
        "image": max_abs(on_image, off_image),
        "eef": max_abs(on_eef, off_eef),
    }
    env.close()
    return {
        "during_read": during,
        "same_state_step_with_audit_toggled": paired,
        "passed": max(during.values()) == 0.0 and max(paired.values()) < 1e-8,
    }


def level_zero() -> dict:
    env = open_env(0)
    try:
        result = {
            "negative": negative(env),
            "robot_perfume": robot_contact(
                env, "bottle_of_perfume_1", "checkrobotcontact::bottle_of_perfume_1"
            ),
            "robot_white_box": robot_contact(
                env, "white_place_box_1", "checkrobotcontact::white_place_box_1"
            ),
            "bowl_perfume": object_contact(
                env,
                "akita_black_bowl_1",
                "bottle_of_perfume_1",
                "checkcontact::akita_black_bowl_1::bottle_of_perfume_1",
            ),
            "bowl_white_box": object_contact(
                env,
                "akita_black_bowl_1",
                "white_place_box_1",
                "checkcontact::akita_black_bowl_1::white_place_box_1",
            ),
        }
        result["success_transition"] = success_step(
            env,
            "akita_black_bowl_1",
            "white_place_box_1",
            "checkcontact::akita_black_bowl_1::white_place_box_1",
        )
        return result
    finally:
        env.close()


def level_one() -> dict:
    env = open_env(1)
    try:
        return {
            "negative": negative(env),
            "robot_toy_car": robot_contact(
                env, "toy_car_1", "checkrobotcontact::toy_car_1"
            ),
            "bowl_toy_car": object_contact(
                env,
                "akita_black_bowl_1",
                "toy_car_1",
                "checkcontact::akita_black_bowl_1::toy_car_1",
            ),
        }
    finally:
        env.close()


def main() -> None:
    import torch

    torch.cuda.init()
    payload = {
        "role": "diagnostic",
        "include_in_baseline": False,
        "benchmark_modified": False,
        "read_only": read_only(),
        "L0": level_zero(),
        "L1": level_one(),
    }
    checks = [
        payload["read_only"]["passed"],
        payload["L0"]["negative"]["audited_clear"],
        payload["L0"]["robot_perfume"]["isolated"],
        payload["L0"]["robot_white_box"]["isolated"],
        payload["L0"]["bowl_perfume"]["isolated"],
        payload["L0"]["bowl_white_box"]["isolated"],
        payload["L0"]["success_transition"].get("official_cost_empty_on_success", False),
        payload["L1"]["negative"]["audited_clear"],
        payload["L1"]["robot_toy_car"]["isolated"],
        payload["L1"]["bowl_toy_car"]["isolated"],
    ]
    payload["accepted"] = all(checks)
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps(payload, indent=2, sort_keys=True))
    print(json.dumps({"wrote": str(OUT), "accepted": payload["accepted"]}, sort_keys=True))


if __name__ == "__main__":
    main()
